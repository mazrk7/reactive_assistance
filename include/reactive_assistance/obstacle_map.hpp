#ifndef REACTIVE_ASSISTANCE_NS_OBSTACLE_MAP_H
#define REACTIVE_ASSISTANCE_NS_OBSTACLE_MAP_H

#include <string>
#include <vector>

#include <boost/thread.hpp>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>

#include <tf2_ros/buffer.h>

#include <reactive_assistance/react_ass_types.hpp>
#include <reactive_assistance/robot_profile.hpp>
#include <reactive_assistance/obstacle.hpp>
#include <reactive_assistance/gap.hpp>
#include <reactive_assistance/trajectory.hpp>

namespace reactive_assistance 
{
  // Represents the map construct for maintaining obstacles and gaps based on laser scan data
  class ObstacleMap 
  {
    public:
      // Constructor & destructor
      ObstacleMap(tf2_ros::Buffer &tf, const RobotProfile &rp);
      ~ObstacleMap() {}

      // Callback for laser scan
      void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);

      // Return the closest gap from in_gaps according to either the angular or Euclidean distance
      GapPtr findClosestGap(const Trajectory &traj, const std::vector<Gap> &in_gaps, bool euclid, int &idx) const;
      // Find whether an input 'gap' is admissible or not, and return the vectors of "virtual" gaps and their clearances
      void findVirtualGaps(const Gap &gap, std::vector<GapPtr> &virt_gaps, std::vector<double> &clearances) const;
      // Compute sub-goal associated with the input gap
      void findSubGoal(const Gap &gap, geometry_msgs::Point &sub_goal) const;

      // Check for safety in navigating a trajectory around a provided list of 'obstacles' and return the list of colliding obstacles
      bool isNavigable(const Trajectory &traj, const std::vector<Obstacle> &obstacles, std::vector<Obstacle> &coll_obstacles) const;

      // Getter for the obstacles detected in the environment
      const std::vector<Obstacle> &getObstacles() const { return obstacles_; }
      // Getter for the gaps detected in the environment
      const std::vector<Gap> &getGaps() const { return gaps_; }
      // Getter for the closest obstacle distance
      double getClosestDistance() const { return min_obs_dist_; }

    private:
      // Compute the obstacles in the environment based on scanner readings
      void updateObstacles();
      // Performs the gap search either clockwise/counterclockwise dependening on right/left
      void gapSearch(const Obstacle &obs, int n, bool right, std::vector<Gap> &gaps, int &next_ind) const;
      // Compute the gaps based on the 'obstacles' surrounding the robot
      void updateGaps();
      // Filter out 'in_gaps' that are duplicates or do not exceed the min gap width and return filtered 'out_gaps'
      void filterGaps(const std::vector<Gap> &in_gaps, std::vector<Gap> &out_gaps) const;
      // Compute clearance to obstacles while traversing a gap via an input trajectory
      double computeClearance(const Trajectory &traj) const;

      tf2_ros::Buffer &tf_buffer_;

      // Robot footprint and kinematic constraints
      RobotProfile robot_profile_;

      // Robot base frame
      std::string robot_frame_;

      // Closest obstacle distance
      double min_obs_dist_;

      // Scan and mutex objects
      boost::mutex scan_mutex_;
      sensor_msgs::LaserScan scan_;

      // Vectors of obstacles and gaps
      std::vector<Obstacle> obstacles_;
      std::vector<Gap> gaps_;

      ros::Subscriber laser_sub_;

      ros::Publisher gaps_pub_;
      ros::Publisher virt_gaps_pub_;
      ros::Publisher closest_gap_pub_;
  };
} /* namespace reactive_assistance */
           
#endif