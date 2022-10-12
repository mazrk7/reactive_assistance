#ifndef REACTIVE_ASSISTANCE_NS_OBSTACLE_H
#define REACTIVE_ASSISTANCE_NS_OBSTACLE_H

#include <geometry_msgs/Point.h>

namespace reactive_assistance 
{
  // Represents an obstacle that could collide with the mobile robot
  class Obstacle
  { 
    public:
      Obstacle(const geometry_msgs::Point& p, double ang, double dist) 
              : point(p)
              , angle(ang)
              , distance(dist) 
      {}
      ~Obstacle() {}

      // Defined by a point, an angle and a distance wrt the robot base
      geometry_msgs::Point point;
      double angle;
      double distance;
  };
} /* namespace reactive_assistance */
     
#endif