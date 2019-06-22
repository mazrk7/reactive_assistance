#include <cmath>
#include <limits>

#include <geometry_msgs/PointStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <reactive_assistance/dist_util.hpp>
// All the other necessary headers included in the class declaration files
#include <reactive_assistance/obstacle_map.hpp>

//==============================================================================
// OBSTACLE MAP DEFINITIONS
//==============================================================================
namespace reactive_assistance 
{
  //==============================================================================
  // PUBLIC OBSTACLE MAP METHODS
  //==============================================================================
  ObstacleMap::ObstacleMap(tf2_ros::Buffer& tf, const RobotProfile& rp) 
                              : tf_buffer_(tf),
                                robot_profile_(rp)        
  {
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    nh_priv.param<std::string>("base_frame", robot_frame_, std::string("base_link"));

    std::string laser_sub_topic;
    nh_priv.param<std::string>("laser_sub_topic", laser_sub_topic, std::string("scan"));
    laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>(laser_sub_topic.c_str(), 1, &ObstacleMap::scanCallback, this);

    // Topics and publishers for gap visualisation
    std::string gaps_pub_topic, virt_gaps_pub_topic, closest_gap_pub_topic;
    nh_priv.param<std::string>("gaps_pub_topic", gaps_pub_topic, std::string("gaps"));
    nh_priv.param<std::string>("virt_gaps_pub_topic", virt_gaps_pub_topic, std::string("virt_gaps"));
    nh_priv.param<std::string>("closest_gap_pub_topic", closest_gap_pub_topic, std::string("closest_gap"));

    gaps_pub_ = nh.advertise<PointCloud>(gaps_pub_topic, 10); 
    virt_gaps_pub_ = nh.advertise<PointCloud>(virt_gaps_pub_topic, 10); 
    closest_gap_pub_ = nh.advertise<PointCloud>(closest_gap_pub_topic, 10); 
  }

  void ObstacleMap::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  { 
    boost::mutex::scoped_lock lock(scan_mutex_);
    scan_ = *scan;

    updateObstacles();
    updateGaps();
  }
  
  // Return the closest gap from in_gaps according to either the angular or Euclidean distance
  GapPtr ObstacleMap::findClosestGap(const Trajectory& traj, const std::vector<Gap>& in_gaps, bool euclid, int& idx) const
  {
    unsigned int gaps_size = in_gaps.size();

    if (gaps_size == 0)
      return NULL;

    double min_dist = std::numeric_limits<double>::max();
    int closest_ind = -1;
    bool close_right = false;
    for (unsigned int i = 0; i < gaps_size; i++) 
    {
      // Right and left side distances of gap
      double dist_rs, dist_ls;
      if (euclid)
      {
        dist_rs = dist(traj.getGoalPoint(), in_gaps[i].right.point);
        dist_ls = dist(traj.getGoalPoint(), in_gaps[i].left.point);
      }
      else
      {
        dist_rs = std::abs(proj(traj.getDirection() - in_gaps[i].right.angle));
        dist_ls = std::abs(proj(traj.getDirection() - in_gaps[i].left.angle));
      }

      if (dist_rs < min_dist || dist_ls < min_dist)
      {  
        closest_ind = i;

        if (dist_rs > dist_ls)
        {
          min_dist = dist_ls;
          close_right = false;
        }
        else
        {
          min_dist = dist_rs;
          close_right = true;
        }
      }
    }

    idx = closest_ind;

    // No closest gap found?
    if (closest_ind == -1)
      return NULL;

    // Initialise point cloud to visualise the candidate closest gaps
    PointCloudPtr point_cloud (new PointCloud);
    point_cloud->header.frame_id = robot_frame_;
    
    point_cloud->points.push_back(pcl::PointXYZ(in_gaps[closest_ind].right.point.x, in_gaps[closest_ind].right.point.y, in_gaps[closest_ind].right.point.z));
    point_cloud->points.push_back(pcl::PointXYZ(in_gaps[closest_ind].left.point.x, in_gaps[closest_ind].left.point.y, in_gaps[closest_ind].left.point.z));
    
    closest_gap_pub_.publish(point_cloud);
    
    return GapPtr(new Gap(in_gaps[closest_ind].right, in_gaps[closest_ind].left, close_right));
  }

  // Find whether an input 'gap' is admissible or not, and return the vectors of "virtual" gaps and their clearances
  void ObstacleMap::findVirtualGaps(const Gap& gap, std::vector<GapPtr>& virt_gaps, std::vector<double>& clearances) const
  {   
    // Looping check variable
    bool valid_gap_found = false;
    // Initialise with input gap
    virt_gaps.push_back(GapPtr(new Gap(gap)));

    // Initialise point cloud to visualise the virtual gaps
    PointCloudPtr point_cloud (new PointCloud);
    point_cloud->header.frame_id = robot_frame_;

    do
    {
      // Take the last virtual gap constructed and run it through the iterative algorithm
      GapPtr virt = virt_gaps.back();

      point_cloud->points.push_back(pcl::PointXYZ(virt->right.point.x, virt->right.point.y, virt->right.point.z));
      point_cloud->points.push_back(pcl::PointXYZ(virt->left.point.x, virt->left.point.y, virt->left.point.z));

      std::vector<Obstacle> o_in, o_ex;
      int i = 0;
      // Compute interior and exterior obstacle points
      for (std::vector<Obstacle>::const_iterator it = obstacles_.begin(); it != obstacles_.end(); ++it) 
      {
        if (!almostEqual(it->distance, scan_.range_max))
        {
          if (isBetweenAngles(it->angle, virt->right.angle, virt->left.angle))
            o_in.push_back(*it);
          else
            o_ex.push_back(*it);
        }
        i++;
      }

      // Erase every exterior obstacle point that yields an angular distance of more than PI
      std::vector<Obstacle> o_ex_apo;
      for (std::vector<Obstacle>::const_iterator it = o_ex.begin(); it != o_ex.end(); ++it) 
      {
        if ((proj(it->angle - virt->right.angle) > 0.0) || (proj(it->angle - virt->left.angle) < 0.0))
          o_ex_apo.push_back(*it);
      }

      // Work out the trajectory to this gap's sub goal
      geometry_msgs::Point sub_goal;
      findSubGoal(*virt, sub_goal);

      Trajectory traj(sub_goal);

      // Update clearances vector for this gap
      clearances.push_back(computeClearance(traj));

      // Vector of colliding obstacles
      std::vector<Obstacle> coll_obs;
      // If the tilda exterior obstacles are empty then check the interior
      if (isNavigable(traj, o_ex_apo, coll_obs))
      {
        if (!isNavigable(traj, o_in, coll_obs))
          virt_gaps.push_back(NULL);

        valid_gap_found = true;
      }
      // Else create a new virtual gap!
      else
      {
        unsigned int coll_size = coll_obs.size();
        double min_d = std::numeric_limits<double>::max();
        int close_ind = -1;

        // Find first side of the virtual gap as colliding obstacle point closest to trajectory to gap
        for (unsigned int i = 0; i < coll_size; i++) 
        {
          // Closest obstacle point along trajectory to sub goal of gap
          geometry_msgs::Point p;

          traj.getClosestPoint(coll_obs[i].point, p);
          double closest_dist = dist(coll_obs[i].point, p);
          if (closest_dist < min_d)
          {
            min_d = closest_dist;
            close_ind = i;
          }
        }

        Obstacle first(coll_obs[close_ind].point, coll_obs[close_ind].angle, coll_obs[close_ind].distance);

        // Orientation to mid point in frame M
        double mid_angle = std::atan2(virt->mid.y, virt->mid.x);

        // Origin at (0, 0)
        geometry_msgs::Point org;
        org.x = org.y = org.z = 0.0;

        // Transforms by frame M (for mid)
        geometry_msgs::Point trans_f = first.point;
        transformPoint(org, mid_angle, trans_f);
        double trans_fangle = std::atan2(trans_f.y, trans_f.x);

        geometry_msgs::Point trans_r = virt->right.point;
        transformPoint(org, mid_angle, trans_r);
        double trans_rangle = std::atan2(trans_r.y, trans_r.x);

        geometry_msgs::Point trans_l = virt->left.point;
        transformPoint(org, mid_angle, trans_l);
        double trans_langle = std::atan2(trans_l.y, trans_l.x);

        // Add left and right gaps to the new tilda obstacle vector
        std::vector<Obstacle> o_ex_tild = o_ex;

        close_ind = -1;
        double gamma, beta, dist_ex;
        // First point located to the left (W+), proceed clockwise from right
        if (trans_f.y >= 0.0)
        {
          min_d = dist(virt->right.point, first.point);
          o_ex_tild.push_back(virt->left);

          gamma = trans_fangle - trans_rangle;
          for (unsigned int i = 0; i < o_ex_tild.size(); ++i) 
          {
            geometry_msgs::Point trans_p = o_ex_tild[i].point;
            transformPoint(org, mid_angle, trans_p);

            beta = trans_fangle - std::atan2(trans_p.y, trans_p.x);
            dist_ex = dist(o_ex_tild[i].point, first.point);
            if ((gamma < beta) && (beta < M_PI) && (dist_ex < min_d))
            {
              min_d = dist_ex;
              close_ind = i;
            }
          }

          if (close_ind == -1)
          {
            Obstacle other(virt->right.point, virt->right.angle, virt->right.distance);
            virt_gaps.push_back(GapPtr(new Gap(other, first)));
          }
          else
          {
            Obstacle other(o_ex_tild[close_ind].point, o_ex_tild[close_ind].angle, o_ex_tild[close_ind].distance);
            virt_gaps.push_back(GapPtr(new Gap(other, first)));
          }
        }
        // First point located to the right (W-), proceed counterclockwise from left
        else
        {
          min_d = dist(virt->left.point, first.point);
          o_ex_tild.push_back(virt->right);

          gamma = trans_langle - trans_fangle;
          for (unsigned int i = 0; i < o_ex_tild.size(); ++i) 
          {
            geometry_msgs::Point trans_p = o_ex_tild[i].point;
            transformPoint(org, mid_angle, trans_p);

            beta = std::atan2(trans_p.y, trans_p.x) - trans_fangle;
            dist_ex = dist(o_ex_tild[i].point, first.point);
            if ((gamma < beta) && (beta < M_PI) && (dist_ex < min_d))
            {
              min_d = dist_ex;
              close_ind = i;
            }
          }

          if (close_ind == -1)
          {
            Obstacle other(virt->left.point, virt->left.angle, virt->left.distance);
            virt_gaps.push_back(GapPtr(new Gap(first, other)));
          }
          else
          {
            Obstacle other(o_ex_tild[close_ind].point, o_ex_tild[close_ind].angle, o_ex_tild[close_ind].distance);
            virt_gaps.push_back(GapPtr(new Gap(first, other)));
          }
        }
      }
    } while(!valid_gap_found);
    
    virt_gaps_pub_.publish(point_cloud);
  }

  // Compute sub-goal associated with the input gap
  void ObstacleMap::findSubGoal(const Gap& gap, geometry_msgs::Point& sub_goal) const
  {
    double ds;
    if (gap.width > (2*(robot_profile_.radius + robot_profile_.d_safe)))
      ds = robot_profile_.radius + robot_profile_.d_safe;
    else
      ds = 0.5 * gap.width;

    // Construct a trajectory to the gap mid point
    Trajectory mid_traj(gap.mid);

    // Point along trajectory to mid point that are closest to left/right gap side points
    geometry_msgs::Point pl, pr;
    mid_traj.getClosestPoint(gap.left.point, pl);
    mid_traj.getClosestPoint(gap.right.point, pr);

    // Circumventing point
    geometry_msgs::Point pc;
    // +/- 1 depending on if pc is a left/right gap
    int gamma;
    if ((dist(pl, gap.left.point) > ds) && (dist(pr, gap.right.point) > ds))
    {
      // Closest gap point
      pc = (gap.close_right) ? gap.right.point : gap.left.point;
      gamma = (gap.close_right) ? -1 : 1;
    }
    else
    {
      bool arc_left = (mid_traj.getLengthArc(pl) <= mid_traj.getLengthArc(pr));

      // Closest robot point
      pc = (arc_left) ? gap.left.point : gap.right.point;
      gamma = (arc_left) ? 1 : -1;
    }

    // Compute tangent point coords, for circles S or C
    geometry_msgs::Point pt1, pt2;
    pt1.z = pt2.z = 0.0;

    // Tangent point radii
    double r1, r2;

    // Circle C is if the robot is located within the safe circle S
    if (std::hypot(pc.x, pc.y) <= ds)
    {
      double rad_c = std::hypot(pc.x, pc.y);

      // Angle to arc fixed
      double th = 0.25 * M_PI;

      pt1.x = rad_c*std::sin(th);
      pt2.x = rad_c*std::sin(-th);
      pt1.y = pt2.y = -rad_c*(1-std::cos(th));

      r1 = r2 = rad_c;
    }
    else 
    {
      double circle_eq = pc.x*pc.x + pc.y*pc.y - ds*ds;
      r1 = circle_eq / (2*(pc.y + ds));
      r2 = circle_eq / (2*(pc.y - ds));

      if (almostEqual(pc.y, -ds))
      {
        pt1.x = pt2.x = pc.x;
        pt1.y = pt2.y = 0.0;
      }
      else
      {
        double mag1 = std::hypot(pc.x, pc.y - r1);
        double mag2 = std::hypot(pc.x, pc.y - r2);

        pt1.x = (pc.x/mag1) * std::abs(r1);
        pt1.y = r1 + ((pc.y - r1)/mag1)*std::abs(r1);

        pt2.x = (pc.x/mag2) * std::abs(r2);
        pt2.y = r2 + ((pc.y - r2)/mag2)*std::abs(r2);      
      }
    }

    // Sub-goal calculation from virtual trajectories
    Trajectory tang_traj1(pt1, r1);
    Trajectory tang_traj2(pt2, r2);
    Trajectory pc_traj(pc);
    if ((proj(tang_traj1.getDirection() - pc_traj.getDirection()) * gamma) < 0)
      sub_goal = pt1;
    else if ((proj(tang_traj2.getDirection() - pc_traj.getDirection()) * gamma) < 0)
      sub_goal = pt2;
    else
      sub_goal = gap.mid;
  }

  // Check for safety in navigating a trajectory around a provided list of 'obstacles' and return the list of colliding obstacles
  bool ObstacleMap::isNavigable(const Trajectory& traj, const std::vector<Obstacle>& obstacles, std::vector<Obstacle>& coll_obstacles) const
  {     
    // Origin of circle at (0, r)
    geometry_msgs::Point c;
    c.x = c.z = 0.0;
    c.y = traj.getRadius();
    geometry_msgs::Point goal = traj.getGoalPoint();

    // Saves computation
    int sgnx = sgn(goal.x);
    int delta = (sgnx == sgn(goal.y)) ? 1 : -1;

    // Transformation matrix to goal point orientation
    double Ra, Rb, Rc, Rd;
    Ra = Rd = (goal.x*goal.x - goal.y*goal.y)/(goal.x*goal.x + goal.y*goal.y);
    Rb = -(2*goal.x*goal.y)/(goal.x*goal.x + goal.y*goal.y);
    Rc = -Rb;

    int footprint_length = robot_profile_.footprint.size();
    double min_x, max_x;
    min_x = max_x = robot_profile_.footprint[0].x;
    double min_y, max_y;
    min_y = max_y = robot_profile_.footprint[0].y;  
    // Get min x & y, and max x & y
    for (int i = 1; i < footprint_length; i++) 
    {
      if (min_x > robot_profile_.footprint[i].x)
        min_x = robot_profile_.footprint[i].x;
      if (max_x < robot_profile_.footprint[i].x)
        max_x = robot_profile_.footprint[i].x;
      if (min_y > robot_profile_.footprint[i].y)
        min_y = robot_profile_.footprint[i].y;
      if (max_y < robot_profile_.footprint[i].y)
        max_y = robot_profile_.footprint[i].y;
    }

    // Populate new vector of obstacles outside the boundaries of the robot footprint 
    std::vector<Obstacle> outer_obstacles;
    for (std::vector<Obstacle>::const_iterator it = obstacles.begin(); it != obstacles.end(); ++it) 
    {
      if ((it->point.x < min_x) || (it->point.x > max_x) || (it->point.y < min_y) || (it->point.y > max_y))
        outer_obstacles.push_back(*it);
    }

    // Loop over each edge of robot polygon shape
    for (int i = 0; i < footprint_length; ++i) 
    {
      // Loop over obstacles to find colliding ones
      for (std::vector<Obstacle>::iterator it = outer_obstacles.begin(); it != outer_obstacles.end(); ++it) 
      {
        int next = (i+1) % footprint_length;

        // Intersecting point of edge with obstacle
        geometry_msgs::Point pe;
        // New intersecting point of edge with gap goal point reached
        geometry_msgs::Point pe_star;

        pe.z = pe_star.z = 0.0;
        if (almostEqual(goal.y, 0.0))
        {
          // Start of line to obstacle point is shifted along the horizontal axis
          geometry_msgs::Point start;
          start.x = start.z = 0.0;
          start.y = it->point.y;

          // Check whether the line parallel to the trajectory and positioned by an obstacle point intersects the edge line
          if (lineIntersect(start, it->point, robot_profile_.footprint[i], robot_profile_.footprint[next], pe))
          {
            pe_star.x = pe.x + goal.x;
            pe_star.y = pe.y + goal.y;

            if ((sgnx*pe.x <= sgnx*it->point.x) && (sgnx*it->point.x <= sgnx*pe_star.x))
              coll_obstacles.push_back(*it);
          }
        }
        else
        {
          // Check if the edge side intersects with the circle defined by the obstacle point
          if (circleIntersect(robot_profile_.footprint[i], robot_profile_.footprint[next], c, dist(c, it->point), pe))
          {
            pe_star.x = (Ra*pe.x + Rb*pe.y) + goal.x;
            pe_star.y = (Rc*pe.x + Rd*pe.y) + goal.y;

            // Frame F for intersecting edge point
            double th = std::atan2(pe.y - traj.getRadius(), pe.x);

            geometry_msgs::Point trans_obs = it->point;
            transformPoint(c, th, trans_obs);
            geometry_msgs::Point trans_pe = pe_star;
            transformPoint(c, th, trans_pe);

            if (mod2pi(delta*std::atan2(trans_obs.y, trans_obs.x)) <= mod2pi(delta*std::atan2(trans_pe.y, trans_pe.x)))
              coll_obstacles.push_back(*it);
          }
        } 
      }
    }
    
    if (coll_obstacles.size() > 0)
      return false;
    else
      return true;
  }

  //==============================================================================
  // PRIVATE OBSTACLE MAP METHODS (Utilities)
  //============================================================================== 
  void ObstacleMap::updateObstacles()
  {
    // Get appropriate transform
    geometry_msgs::TransformStamped transform;
    try
    {
        transform = tf_buffer_.lookupTransform(
                      robot_frame_,
                      scan_.header.frame_id,
                      ros::Time(0),
                      ros::Duration(3.0));
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_ERROR("Error during transform: %s", ex.what());
    }

    std::vector<Obstacle> obstacles;
    unsigned int obs_size = scan_.ranges.size();
    min_obs_dist_ = scan_.ranges[0];
    // Populate the obstacles vector from scanner readings
    for (unsigned int i = 0; i < obs_size; ++i)
    {
      const double& range = scan_.ranges[i];
      double angle = scan_.angle_min + i*scan_.angle_increment;
        
      geometry_msgs::PointStamped base_point;
      geometry_msgs::PointStamped scan_point;
      
      scan_point.point.x = range*std::cos(angle);
      scan_point.point.y = range*std::sin(angle);
      scan_point.point.z = 0.0;
      
      // Transform scan point to base point
      tf2::doTransform(scan_point, base_point, transform);
      if (!std::isinf(range))
        obstacles.push_back(Obstacle(base_point.point, angle, range));
      else
        obstacles.push_back(Obstacle(base_point.point, angle, scan_.range_max));
      
      // Track closest obstacle distance
      if (range < min_obs_dist_)
        min_obs_dist_ = range;
    }

    // Overwrite obstacles vector property
    obstacles_ = obstacles;
  }
  
  void ObstacleMap::gapSearch(const Obstacle& obs, int n, bool right, std::vector<Gap>& gaps, int& next_ind) const
  {
      // Wrap around effect for checking next index
      int next = (right) ? ((next_ind+1) % n) : ((n + (next_ind-1)) % n);

      // Depth discontinuity detected when two contiguous depth measurements are either
      // separated by the min width (bilateral: basis on endpoint closer to robot) OR
      // either measurement is a non-obstacle point (unilateral: basis at unique endpoint)
      if (((dist(obstacles_[next].point, obs.point) > robot_profile_.min_gap_width) && (obs.distance < obstacles_[next].distance))
          || (!almostEqual(obs.distance, scan_.range_max) && almostEqual(obstacles_[next].distance, scan_.range_max)))
      {  
        // Initialise min variables
        double min_visi, min_dist;
        min_visi = min_dist = std::numeric_limits<double>::max();
        int min_ind = -1;

        // Fixed squared distance to gap start point
        double dist_gap = obs.distance * obs.distance;
               
        // Evaluate OTHER side of gap by searching obstacle points falling to the opposite of the found gap side
        int i = next;
        // O+ points are those in which the angular distance does not exceed PI
        bool ang_safe = (right) ? (proj(obstacles_[i].angle - obs.angle) > 0.0) : (proj(obstacles_[i].angle - obs.angle) < 0.0);
        while (ang_safe)
        {
          if (!almostEqual(obstacles_[i].distance, scan_.range_max))
          {
            // Determine whether these O+ points are valid or not
            double distp = dist(obs.point, obstacles_[i].point);
            double visibility = std::acos((dist_gap + distp*distp - obstacles_[i].distance*obstacles_[i].distance) / (2*distp*obs.distance));
            
            // Valid O+ point if visibility condition met
            if (visibility < min_visi)
            {
              min_visi = visibility;
              
              // Find closest point from the valid ones
              if (distp < min_dist)
              {
                min_dist = distp;
                min_ind = i;
              }
            }
          }
          
          // Next point to evaluate and angular safety check
          i = (right) ? ((i+1) % n) : ((n + (i-1)) % n);
          ang_safe = (right) ? (proj(obstacles_[i].angle - obs.angle) > 0.0) : (proj(obstacles_[i].angle - obs.angle) < 0.0);
        }

        // If there is an empty set of valid O+ points
        if (min_ind == -1)
        {
          // Left side is a point at a distance R+d_safe and angle of left neighbourhood
          geometry_msgs::Point virtual_point;
          double virt_safe = robot_profile_.radius+robot_profile_.d_safe;
          virtual_point.x = obs.point.x + virt_safe*std::cos(obstacles_[next].angle);
          virtual_point.y = obs.point.y + virt_safe*std::sin(obstacles_[next].angle);
          virtual_point.z = 0.0;

          // Law of cosines for distance to virtual point
          double range = std::sqrt(virt_safe*virt_safe + dist_gap - 2*virt_safe*obs.distance*std::cos(obstacles_[next].angle - obs.angle));

          if (right)
            gaps.push_back(Gap(obs, Obstacle(virtual_point, obstacles_[next].angle, range))); 
          else
            gaps.push_back(Gap(Obstacle(virtual_point, obstacles_[next].angle, range), obs)); 

          // Resume scanning from left neighbour
          next_ind = next;
        }
        else
        {
          if (right)
          {
            // Add gap to the vector with the basis right side and determined left side
            gaps.push_back(Gap(obs, obstacles_[min_ind])); 

            // Resume scanning from left side, unless it exceeds last sensor point
            next_ind = (min_ind < next_ind) ? 0 : min_ind;
          }
          else
          {
            // Add gap to the vector with the basis right side and determined left side
            gaps.push_back(Gap(obstacles_[min_ind], obs)); 

            // Resume scanning from right side, unless it exceeds last sensor point
            next_ind = (min_ind > next_ind) ? (n-1) : min_ind;
          }
        }
      }
      else
      {
        next_ind = next;
      }
  }

  // Admissible Gap method of evaluating each range reading to detect gaps (treating each scan as a sector)
  void ObstacleMap::updateGaps()
  { 
    std::vector<Gap> gaps;
    int n = obstacles_.size();

    // Counterclockwise search is to check for the existence of RIGHT discontinuities
    int k = 0;
    do
    {
      gapSearch(obstacles_[k], n, true, gaps, k);
    } while(k != 0);
    
    // Clockwise search is to check for the existence of LEFT discontinuities
    k = n-1;
    do
    {
      gapSearch(obstacles_[k], n, false, gaps, k);
    } while(k != (n-1));

    // Filter the gaps detected
    std::vector<Gap> filt_gaps;
    filterGaps(gaps, filt_gaps);

    // Overwrite gaps property
    gaps_ = filt_gaps;
  }
  
  // Filter out gaps to eliminate duplicates and gaps that do not exceed the required width
  void ObstacleMap::filterGaps(const std::vector<Gap>& in_gaps, std::vector<Gap>& out_gaps) const
  {
    std::vector<Gap> filt_gaps;

    // Initialise point cloud to visualise the gaps
    PointCloudPtr point_cloud (new PointCloud);
    point_cloud->header.frame_id = robot_frame_;
    
    unsigned int gaps_size = in_gaps.size();
    // Evaluate each gap to determine whether to eliminate it if it exists within another gap
    for (unsigned int i = 0; i < gaps_size; ++i)
    {     
      bool redundant_gap = false;

      // If the ith is a 'front' gap
      if (in_gaps[i].front)
      {
        for (unsigned int j = 0; j < gaps_size; ++j)
        {        
          if (in_gaps[j].front && ((i != j) 
            && (in_gaps[i].right.angle >= in_gaps[j].right.angle) 
            && (in_gaps[i].left.angle <= in_gaps[j].left.angle)))
          {
            redundant_gap = true;
            break;
          }
        }
      }
      else
      {
        for (unsigned int j = 0; j < gaps_size; ++j)
        {        
          if (!in_gaps[j].front && ((i != j) 
            && (proj(in_gaps[i].right.angle-M_PI) >= proj(in_gaps[j].right.angle-M_PI))
            && (proj(in_gaps[i].left.angle-M_PI) <= proj(in_gaps[j].left.angle-M_PI))))
          {
            redundant_gap = true;
            break;
          }
        }
      }

      if (!redundant_gap) 
        filt_gaps.push_back(in_gaps[i]);      
    }
    
    gaps_size = filt_gaps.size();
    // Make sure filtered gaps fulfil the minimum width requirement
    for (unsigned int i = 0; i < gaps_size; ++i)
    {
      if (filt_gaps[i].width > robot_profile_.min_gap_width)
      {
        out_gaps.push_back(filt_gaps[i]);

        point_cloud->points.push_back(pcl::PointXYZ(out_gaps[i].right.point.x, out_gaps[i].right.point.y, out_gaps[i].right.point.z));
        point_cloud->points.push_back(pcl::PointXYZ(out_gaps[i].left.point.x, out_gaps[i].left.point.y, out_gaps[i].left.point.z));
      }
    }
    
    if (out_gaps.size() > 0)
      gaps_pub_.publish(point_cloud);
  }

  // Compute clearance to obstacles while traversing a gap via an input trajectory
  double ObstacleMap::computeClearance(const Trajectory& traj) const
  {     
    unsigned int obs_size = obstacles_.size();
    double min_d = std::numeric_limits<double>::max();

    for (unsigned int i = 0; i < obs_size; i++) 
    {
      geometry_msgs::Point p;

      traj.getClosestPoint(obstacles_[i].point, p);
      double distp = dist(obstacles_[i].point, p);

      if (distp < min_d)
        min_d = distp;
    }

    return min_d;
  }
} /* namespace reactive_assistance */
