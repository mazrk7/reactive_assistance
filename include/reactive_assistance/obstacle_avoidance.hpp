#ifndef REACTIVE_ASSISTANCE_NS_OBSTACLE_AVOIDANCE_H
#define REACTIVE_ASSISTANCE_NS_OBSTACLE_AVOIDANCE_H

#include <string>
#include <vector>

#include <boost/thread.hpp>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <tf2_ros/buffer.h>

#include <reactive_assistance/react_ass_types.hpp>
#include <reactive_assistance/robot_profile.hpp>
#include <reactive_assistance/trajectory.hpp>
#include <reactive_assistance/obstacle_map.hpp>

namespace reactive_assistance 
{
    class ObstacleAvoidance 
    {
      public:
        // Constructor & destructor
        ObstacleAvoidance(tf2_ros::Buffer& tf);
        ~ObstacleAvoidance();

        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
        void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
        void cmdCallback(const geometry_msgs::Twist::ConstPtr& twist);  

      private:
        // Compute motion command to navigate a safe trajectory
        void computeMotionCommand(const Trajectory& safe_traj, geometry_msgs::Twist& assist) const;
        // Find assistive command for the simulated trajectory 'traj'
        void findAssistiveCommand(const Trajectory& traj, geometry_msgs::Twist& assist) const;
        // Return goal point specified by a 'global' planner
        TrajPtr getGlobalTrajectory() const; 
        // Return goal point of simulated trajectory
        TrajPtr simulateTrajectory(const geometry_msgs::Twist& twist_msg) const; 
        // Navigate towards a global goal
        void navigationLoop(double rate) const;

        tf2_ros::Buffer& tf_buffer_;

        // Robot footprint and kinematic constraints
        RobotProfile* robot_profile_;
        // Obstacle map where gaps are computed and navigation functions are performed
        ObstacleMap* obs_map_;

        // Robot base frame and global frame
        std::string robot_frame_;
        std::string world_frame_;

        // Simulation time and discretisation
        double sim_time_;
        double sim_granularity_;

        // Obstacle avoidance  control loop thread
        boost::thread* control_thread_;

        // Odom and mutex objects
        boost::mutex odom_mutex_;
        nav_msgs::Odometry curr_odom_;

        // Global goal pose and flag to confirm availability
        bool available_goal_;
        geometry_msgs::PoseStamped goal_pose_;

        // Publishers & subscribers
        ros::Publisher safe_cmd_pub_;
        ros::Publisher auto_cmd_pub_;
        ros::Publisher traj_pub_;
        ros::Publisher obs_pub_;
        ros::Publisher footprint_pub_;
        ros::Publisher global_goal_pub_;

        ros::Subscriber odom_sub_;
        ros::Subscriber goal_sub_;
        ros::Subscriber cmd_sub_;
  };
} /* namespace reactive_assistance */
           
#endif