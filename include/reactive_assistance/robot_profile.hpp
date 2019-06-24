#ifndef REACTIVE_ASSISTANCE_NS_ROBOT_PROFILE_H
#define REACTIVE_ASSISTANCE_NS_ROBOT_PROFILE_H

#include <vector>

#include <geometry_msgs/Point.h>

namespace reactive_assistance 
{
  // Represents the geometric and kinematic profile of the mobile robot
  class RobotProfile
  {
    public:
      RobotProfile(const std::vector<geometry_msgs::Point>& fp, double r, double dvs, 
                  double min_g, double vx, double vth, double acc_x, double acc_th)
                  : footprint(fp)
                  , radius(r)
                  , d_safe(2*r)
                  , dvel_safe(dvs)
                  , min_gap_width(min_g)
                  , max_vx(vx)
                  , max_vth(vth)
                  , acc_vx_lim(acc_x)
                  , acc_vth_lim(acc_th)
      {}  
      ~RobotProfile() {}

      // Shape of robot approximated by a polygon
      std::vector<geometry_msgs::Point> footprint;

      // Radius of virtual circle wrapped around the robot
      double radius;

      // Suitable clearance to obstacles, m
      double d_safe;

      // How much the speed is limited near obstacles, m
      double dvel_safe;
      
      // Minimum width of an opening that the robot can fit through
      double min_gap_width;

      // Max linear and angular velocities
      double max_vx;
      double max_vth;

      // Limits on the linear and angular accelerations
      double acc_vx_lim;
      double acc_vth_lim;
  };
} /* namespace reactive_assistance */
     
#endif