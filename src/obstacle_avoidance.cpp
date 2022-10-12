#include <cmath>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <reactive_assistance/obstacle.hpp>
#include <reactive_assistance/gap.hpp>
#include <reactive_assistance/dist_util.hpp>
// All the other necessary headers included in the class declaration files
#include <reactive_assistance/obstacle_avoidance.hpp>

namespace reactive_assistance
{
  //==============================================================================
  // PUBLIC OBSTACLE AVOIDANCE METHODS
  //==============================================================================

  ObstacleAvoidance::ObstacleAvoidance(tf2_ros::Buffer &tf)
      : tf_buffer_(tf), robot_profile_(NULL), obs_map_(NULL), control_thread_(NULL), available_goal_(false)
  {
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    nh_priv.param<std::string>("base_frame", robot_frame_, std::string("base_link"));
    nh_priv.param<std::string>("odom_frame", odom_frame_, std::string("odom"));
    nh_priv.param<std::string>("world_frame", world_frame_, std::string("map"));

    // NOTE: Actually halved width/length dimensions OR virtual circle radius of base footprint
    double fp_wid, fp_len, radius;
    std::string foot_wid_param_name, foot_len_param_name, radius_param_name;
    // Set min gap width depending on robot shape
    double min_gap_width = 0.0;

    // Creating robot footprint as a vector of points forming a polygon shape, depending on base shape
    bool rectangular_base = nh_priv.searchParam("footprint_width", foot_wid_param_name) && nh_priv.searchParam("footprint_length", foot_len_param_name);
    bool circular_base = nh_priv.searchParam("radius", radius_param_name);
    nh_priv.getParam(foot_wid_param_name, fp_wid);
    nh_priv.getParam(foot_len_param_name, fp_len);
    nh_priv.getParam(radius_param_name, radius);
    if (rectangular_base && circular_base)
    {
      ROS_ERROR("Cannot specify both a rectangular and circular robot base!");
      ROS_ERROR("footprint_width: %.3f", fp_wid);
      ROS_ERROR("footprint_length: %.3f", fp_len);
      ROS_ERROR("radius: %.3f", radius);
      // Shutdown node
      ros::shutdown();
    }
    else if (rectangular_base)
    {
      ROS_INFO("Rectangular robot with halved footprint width %.3f and length %.3f", fp_wid, fp_len);
      radius = fp_wid;
      ROS_INFO("Virtual radius set to half footprint width: %.3f", radius);
      min_gap_width = 2.0 * fp_wid;
    }
    else if (circular_base)
    {
      ROS_INFO("Circular robot with radius %.3f", radius);
      fp_wid = radius;
      min_gap_width = 2.0 * radius;
    }
    else
    {
      ROS_ERROR("No base footprint shape parameters set, choosing default circular values!");
      radius = 0.4;
      min_gap_width = 2.0 * radius;
    }

    ROS_INFO("Minimum gap width: %.3f", min_gap_width);

    std::vector<geometry_msgs::Point> footprint;
    if (rectangular_base)
    {
      geometry_msgs::Point bleft;
      bleft.x = -fp_len;
      bleft.y = -fp_wid;
      footprint.push_back(bleft);

      geometry_msgs::Point bright;
      bright.x = -fp_len;
      bright.y = fp_wid;
      footprint.push_back(bright);

      geometry_msgs::Point tright;
      tright.x = fp_len;
      tright.y = fp_wid;
      footprint.push_back(tright);

      geometry_msgs::Point tleft;
      tleft.x = fp_len;
      tleft.y = -fp_wid;
      footprint.push_back(tleft);
    }
    else
    {
      // Loop over 8 angles around a circle making a point each time
      int N = 8;
      geometry_msgs::Point pt;
      for (int i = 0; i < N; ++i)
      {
        double angle = i * M_2PI / N;
        pt.x = cos(angle) * radius;
        pt.y = sin(angle) * radius;

        footprint.push_back(pt);
      }
    }

    // Limit safety speed distance
    double dvel_safe;
    nh_priv.param<double>("dvel_safe", dvel_safe, 0.9);

    double max_vx, max_vth, acc_x, acc_th;
    nh_priv.param<double>("max_lin_vel", max_vx, 1.0);
    nh_priv.param<double>("max_ang_vel", max_vth, 1.0);
    nh_priv.param<double>("acc_vx_lim", acc_x, 1.0);
    nh_priv.param<double>("acc_vth_lim", acc_th, 1.0);

    robot_profile_ = new RobotProfile(footprint, radius, dvel_safe, min_gap_width, max_vx, max_vth, acc_x, acc_th);
    ROS_INFO_STREAM("Loaded the robot profile...");

    obs_map_ = new ObstacleMap(tf_buffer_, *robot_profile_);
    ROS_INFO_STREAM("Loaded the obstacle map...");

    nh_priv.param<double>("sim_time", sim_time_, 1.0);
    nh_priv.param<double>("sim_granularity", sim_granularity_, 0.1);

    double control_rate;
    nh_priv.param<double>("control_rate", control_rate, 10);
    // Set up the autonomous control thread
    control_thread_ = new boost::thread(boost::bind(&ObstacleAvoidance::navigationLoop, this, control_rate));

    // Publishers & Subscribers
    std::string safe_cmd_pub_topic, auto_cmd_pub_topic, traj_pub_topic, obs_pub_topic, footprint_pub_topic, goal_pub_topic;
    nh_priv.param<std::string>("safe_cmd_pub_topic", safe_cmd_pub_topic, std::string("cmd_vel"));
    nh_priv.param<std::string>("auto_cmd_pub_topic", auto_cmd_pub_topic, std::string("auto_vel"));
    nh_priv.param<std::string>("traj_pub_topic", traj_pub_topic, std::string("desired_traj"));
    nh_priv.param<std::string>("obs_pub_topic", obs_pub_topic, std::string("collision_obstacles"));
    nh_priv.param<std::string>("footprint_pub_topic", footprint_pub_topic, std::string("footprint"));
    nh_priv.param<std::string>("goal_pub_topic", goal_pub_topic, std::string("nav_goal"));

    safe_cmd_pub_ = nh.advertise<geometry_msgs::Twist>(safe_cmd_pub_topic, 10);
    auto_cmd_pub_ = nh.advertise<geometry_msgs::Twist>(auto_cmd_pub_topic, 10);

    // Below publishers are for debugging/visualisation purposes
    traj_pub_ = nh.advertise<geometry_msgs::PoseArray>(traj_pub_topic, 10);
    obs_pub_ = nh.advertise<PointCloud>(obs_pub_topic, 10);
    footprint_pub_ = nh.advertise<visualization_msgs::Marker>(footprint_pub_topic, 10);
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(goal_pub_topic, 10);

    std::string odom_sub_topic, goal_sub_topic, cmd_sub_topic;
    nh_priv.param<std::string>("odom_sub_topic", odom_sub_topic, std::string("odom"));
    nh_priv.param<std::string>("goal_sub_topic", goal_sub_topic, std::string("goal"));
    nh_priv.param<std::string>("cmd_sub_topic", cmd_sub_topic, std::string("input_vel"));

    odom_sub_ = nh.subscribe<nav_msgs::Odometry>(odom_sub_topic.c_str(), 1, &ObstacleAvoidance::odomCallback, this);
    goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(goal_sub_topic.c_str(), 1, &ObstacleAvoidance::goalCallback, this);
    cmd_sub_ = nh.subscribe<geometry_msgs::Twist>(cmd_sub_topic.c_str(), 1, &ObstacleAvoidance::cmdCallback, this);

    ROS_INFO_STREAM("Loaded the obstacle avoidance...");
  }

  ObstacleAvoidance::~ObstacleAvoidance()
  {
    if (robot_profile_ != NULL)
    {
      delete robot_profile_;
    }

    if (obs_map_ != NULL)
    {
      delete obs_map_;
    }

    if (control_thread_ != NULL)
    {
      control_thread_->join();
      delete control_thread_;
    }
  }

  void ObstacleAvoidance::odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
  {
    boost::mutex::scoped_lock lock(odom_mutex_);
    curr_odom_ = *odom;

    // Create footprint polygon visualisation as a list of lines
    visualization_msgs::Marker line_list;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.header.stamp = ros::Time::now();
    line_list.header.frame_id = odom_frame_;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose = curr_odom_.pose.pose;
    line_list.scale.x = 0.03;

    // Coloured green 
    line_list.color.g = 1.0;
    line_list.color.a = 1.0;

    int fp_length = robot_profile_->footprint.size();
    // Loop over each edge of robot polygon shape
    for (int i = 0; i < fp_length; ++i)
    {
      int next = (i + 1) % fp_length;
      line_list.points.push_back(robot_profile_->footprint[i]);
      line_list.points.push_back(robot_profile_->footprint[next]);
    }

    footprint_pub_.publish(line_list);
  }

  void ObstacleAvoidance::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal)
  {
    curr_goal_ = *goal;
    available_goal_ = true;

    goal_pub_.publish(curr_goal_);
    ROS_INFO("Goal pose updated!");
  }

  void ObstacleAvoidance::cmdCallback(const geometry_msgs::Twist::ConstPtr &twist)
  {
    const geometry_msgs::Twist &orig = *twist;

    // Goal trajectory to pursue
    TrajPtr goal_traj;
    // Get simulated goal trajectory if one hasn't been specified by a 'global' source
    if (available_goal_)
    {
      goal_traj = getGlobalTrajectory();
    }
    else
    {
      goal_traj = simulateTrajectory(orig);
    }

    // Assistive command
    geometry_msgs::Twist assist;
    // Colliding obstacles vector
    std::vector<Obstacle> obstacles;

    // Handle different drive scenarios:
    // a) Joystick deadzone or error in transform
    if ((std::abs(orig.linear.x) < 0.1 && std::abs(orig.angular.z) < 0.1) || (goal_traj == NULL))
    {
      assist.linear.x = 0.0;
      assist.angular.z = 0.0;
    }
    // b) Free-path to goal situation
    else if (obs_map_->isNavigable(*goal_traj, obs_map_->getObstacles(), obstacles))
    {
      assist = orig;
    }
    // c) Dangerous-path to goal situation
    else
    {
      PointCloudPtr cloud(new PointCloud);
      cloud->header.frame_id = robot_frame_;

      for (std::vector<Obstacle>::const_iterator it = obstacles.begin(); it != obstacles.end(); ++it)
      {
        cloud->points.push_back(pcl::PointXYZ(it->point.x, it->point.y, 0.0));
      }

      // Publish the colliding obstacles
      obs_pub_.publish(cloud);

      // Find the assistive command
      findAssistiveCommand(*goal_traj, assist);

      ROS_INFO_STREAM("Original: Lin " << orig.linear.x << " Ang " << orig.angular.z);
      ROS_INFO_STREAM("Assisted: Lin " << assist.linear.x << " Ang " << assist.angular.z);
    }

    // Publish the safe navigational command
    safe_cmd_pub_.publish(assist);
  }

  //==============================================================================
  // PRIVATE OBSTACLE AVOIDANCE METHODS (Utilities)
  //==============================================================================

  // Compute motion commands to navigate a safe trajectory
  void ObstacleAvoidance::computeMotionCommand(const Trajectory &safe_traj, geometry_msgs::Twist &assist) const
  {
    // Safe trajectory tangent direction
    double safe_heading = std::atan(1.0 / safe_traj.getRadius());

    // Compute velocity limit
    double vlim = std::sqrt(1.0 - sat((robot_profile_->dvel_safe - obs_map_->getClosestDistance()) / robot_profile_->dvel_safe, 0.0, 1.0)) * robot_profile_->max_vx;

    // Generate motion commands to simulate trajectory
    assist.linear.x = sgn(safe_traj.getGoalPoint().x) * vlim * std::cos(safe_heading);
    assist.angular.z = sgn(safe_traj.getGoalPoint().x) * vlim * std::sin(safe_heading);
  }

  // Find assistive command for the simulated trajectory 'traj'
  void ObstacleAvoidance::findAssistiveCommand(const Trajectory &traj, geometry_msgs::Twist &assist) const
  {
    bool gap_search_fin = false;
    std::vector<Gap> gaps_check = obs_map_->getGaps();

    while (!gap_search_fin)
    {
      // Retrieve the best gap for "free walking"
      GapPtr closest;
      int close_idx;

      // If global plan is available then use Euclidean distance
      closest = obs_map_->findClosestGap(traj, gaps_check, available_goal_, close_idx);

      // If no best gap identified, break loop
      if (closest != NULL)
      {
        // Construct a vector of virtually admissible gaps for navigation
        std::vector<GapPtr> virt_gaps;
        // Clearances to virtual gaps
        std::vector<double> clearances;

        obs_map_->findVirtualGaps(*closest, virt_gaps, clearances);

        // Found an admissible gap
        if (virt_gaps.back() != NULL)
        {
          gap_search_fin = true;

          // Compute clearance max and min values
          double cl_max = *std::max_element(clearances.begin(), clearances.end());
          double cl_min = *std::min_element(clearances.begin(), clearances.end());

          std::vector<double> gap_weights;
          double w_total = 0.0;
          // Loop over virtual gaps and compute weights
          for (unsigned int i = 0; i < virt_gaps.size(); ++i)
          {
            double weight = (cl_max == cl_min) ? 1.0 : sat(1.0 - ((cl_max - clearances[i]) / (cl_max - cl_min)), 0.0, 1.0);
            w_total += (weight * weight);
            gap_weights.push_back(weight);
          }

          geometry_msgs::Point sub_goal;
          double x, y;
          x = y = 0.0;
          // Compute sub goals
          for (unsigned int i = 0; i < virt_gaps.size(); ++i)
          {
            obs_map_->findSubGoal(*virt_gaps[i], sub_goal);
            double relative_weight = ((gap_weights[i] * gap_weights[i]) / w_total);
            x += (relative_weight * sub_goal.x);
            y += (relative_weight * sub_goal.y);
          }

          geometry_msgs::Point avg_goal;
          avg_goal.x = x;
          avg_goal.y = y;
          avg_goal.z = 0.0;

          // Trajectories to the weighted average goal and the last constructed virtual gap goal
          Trajectory avg(avg_goal);
          Trajectory last_sub(sub_goal);

          // Obstacles preventing navigability of the path to the weighted average goal
          std::vector<Obstacle> coll_obstacles;
          if (obs_map_->isNavigable(avg, obs_map_->getObstacles(), coll_obstacles))
          {
            computeMotionCommand(avg, assist);
          }
          else
          {
            computeMotionCommand(last_sub, assist);
          }
        }
        else
        {
          // Delete the non-admissible closest gap and loop through algorithm again
          gaps_check.erase(gaps_check.begin() + close_idx);
        }
      }
      else
      {
        // Non-admissible gap
        assist.linear.x = 0.0;
        assist.angular.z = 0.0;

        gap_search_fin = true;
      }
    }
  }

  TrajPtr ObstacleAvoidance::getGlobalTrajectory() const
  {
    // Transform global goal coordinates to robot frame
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform(
          robot_frame_,
          world_frame_,
          ros::Time(0),
          ros::Duration(3.0));
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_ERROR("Error during transform: %s", ex.what());
      return NULL;
    }

    geometry_msgs::PoseStamped goal_robot;
    tf2::doTransform(curr_goal_, goal_robot, transform);

    return TrajPtr(new Trajectory(goal_robot.pose.position));
  }

  TrajPtr ObstacleAvoidance::simulateTrajectory(const geometry_msgs::Twist &twist_msg) const
  {
    // Transform local odom coordinates to robot frame
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform(
          robot_frame_,
          odom_frame_,
          ros::Time(0),
          ros::Duration(3.0));
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_ERROR("Error during transform: %s", ex.what());
      return NULL;
    }

    // Create trajectory as a pose array
    geometry_msgs::PoseArray traj_cloud;
    traj_cloud.header.frame_id = odom_frame_;

    // Current odometry info
    double x = curr_odom_.pose.pose.position.x;
    double y = curr_odom_.pose.pose.position.y;

    double yaw, _pitch, _roll;
    tf2::Matrix3x3(tf2::Quaternion(curr_odom_.pose.pose.orientation.x, curr_odom_.pose.pose.orientation.y,
                                   curr_odom_.pose.pose.orientation.z, curr_odom_.pose.pose.orientation.w))
        .getEulerYPR(yaw, _pitch, _roll);
    double th = yaw;

    double vx = curr_odom_.twist.twist.linear.x;
    double vth = curr_odom_.twist.twist.angular.z;

    // User's intended commands
    double vel_lin = twist_msg.linear.x;
    double vel_ang = twist_msg.angular.z;

    // Compute the number of steps to project along the trajectory
    int num_steps = std::ceil(sim_time_ / sim_granularity_);
    double dt = sim_time_ / num_steps;

    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::PoseStamped goal_stamped;
    tf2::Quaternion q;
    // Loop over forward simulation steps to generate trajectory
    for (int i = 0; i < num_steps; ++i)
    {
      // Compute updated velocities given current speed
      vx = (vx < vel_lin) ? std::min(vel_lin, vx + robot_profile_->acc_vx_lim * dt) : std::max(vel_lin, vx - robot_profile_->acc_vx_lim * dt);
      vth = (vth < vel_ang) ? std::min(vel_ang, vth + robot_profile_->acc_vth_lim * dt) : std::max(vel_ang, vth - robot_profile_->acc_vth_lim * dt);

      // Compute updated position of robot given the new velocities
      x += (vx * std::cos(th) * dt);
      y += (vx * std::sin(th) * dt);
      th += (vth * dt);

      pose_stamped.pose.position.x = x;
      pose_stamped.pose.position.y = y;
      pose_stamped.pose.position.z = 0.0;

      q.setRPY(0, 0, th);
      tf2::convert(q, pose_stamped.pose.orientation);

      traj_cloud.poses.push_back(pose_stamped.pose);
      tf2::doTransform(pose_stamped, goal_stamped, transform);
    }

    // Publish trajectory of robot
    traj_pub_.publish(traj_cloud);

    // If an invalid circular arc due to a purely rotational motion
    if (almostEqual(goal_stamped.pose.position.y, 0.0) && almostEqual(goal_stamped.pose.position.x, 0.0))
    {
      double y_goal, _p_goal, _r_goal;
      tf2::Matrix3x3(tf2::Quaternion(goal_stamped.pose.orientation.x, goal_stamped.pose.orientation.y,
                                     goal_stamped.pose.orientation.z, goal_stamped.pose.orientation.w))
          .getEulerYPR(y_goal, _p_goal, _r_goal);

      // Assume goal is at rotated edge of robot's virtual radius
      goal_stamped.pose.position.x = robot_profile_->radius * std::cos(y_goal);
      goal_stamped.pose.position.y = robot_profile_->radius * std::sin(y_goal);
    }

    // If invalid due to a straight trajectory, slightly offset the 'y'
    if (almostEqual(goal_stamped.pose.position.y, 0.0))
    {
      goal_stamped.pose.position.y += epsilon;
    }

    // Publish simulated goal of robot trajectory
    goal_pub_.publish(goal_stamped);

    return TrajPtr(new Trajectory(goal_stamped.pose.position));
  }

  bool ObstacleAvoidance::isGoalReached() const
  {
    // Transform global goal coordinates to odom frame
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform(
          odom_frame_,
          world_frame_,
          ros::Time(0),
          ros::Duration(3.0));
    }
    catch (const tf2::TransformException &ex)
    {
      ROS_ERROR("Error during transform: %s", ex.what());
      return false;
    }

    geometry_msgs::PoseStamped curr_goal_stamped;
    curr_goal_stamped.pose = curr_goal_.pose;
    curr_goal_stamped.header.frame_id = world_frame_;

    geometry_msgs::PoseStamped goal_odom;
    tf2::doTransform(curr_goal_stamped, goal_odom, transform);

    return (dist(goal_odom.pose.position, curr_odom_.pose.pose.position) < robot_profile_->radius);
  }

  void ObstacleAvoidance::navigationLoop(double rate)
  {
    ros::NodeHandle nh;
    ros::Rate r(rate);
    while (nh.ok())
    {
      // Check if a goal point is available and whether the position has been reached yet
      if (available_goal_ && isGoalReached())
      {
        ROS_INFO("Reached the published goal pose!");
        available_goal_ = false;

        // Publish a terminating velocity command
        geometry_msgs::Twist zero_twist;
        zero_twist.linear.x = zero_twist.angular.z = 0.0;
        auto_cmd_pub_.publish(zero_twist);
      }

      if (available_goal_)
      {
        // Goal trajectory to pursue
        TrajPtr goal_traj;
        goal_traj = getGlobalTrajectory();

        // Assistive command
        geometry_msgs::Twist assist;
        // Colliding obstacles vector
        std::vector<Obstacle> obstacles;

        computeMotionCommand(*goal_traj, assist);

        if (!obs_map_->isNavigable(*goal_traj, obs_map_->getObstacles(), obstacles))
        {
          PointCloudPtr cloud(new PointCloud);
          cloud->header.frame_id = robot_frame_;

          for (std::vector<Obstacle>::const_iterator it = obstacles.begin(); it != obstacles.end(); ++it)
          {
            cloud->points.push_back(pcl::PointXYZ(it->point.x, it->point.y, 0.0));
          }

          // Publish the colliding obstacles
          obs_pub_.publish(cloud);

          // Find the assistive command
          findAssistiveCommand(*goal_traj, assist);
        }

        // Publish the autonomous navigation command if a goal is still available
        ROS_INFO_STREAM("Autonomous: Lin " << assist.linear.x << " Ang " << assist.angular.z);
        auto_cmd_pub_.publish(assist);
      }

      r.sleep();
    }
  }
} /* namespace reactive_assistance */