#ifndef REACTIVE_ASSISTANCE_NS_OBSTACLE_H
#define REACTIVE_ASSISTANCE_NS_OBSTACLE_H

#include <geometry_msgs/Point.h>

namespace reactive_assistance 
{
  class Obstacle
  { 
    public:
      Obstacle(const geometry_msgs::Point& p, double ang, double dist) 
                : point(p), 
                  angle(ang), 
                  distance(dist) 
                  {} 
      ~Obstacle() {}
      
      geometry_msgs::Point point;
      double angle;
      double distance;
  };
} /* namespace reactive_assistance */
     
#endif