#ifndef REACTIVE_ASSISTANCE_NS_TRAJECTORY_H
#define REACTIVE_ASSISTANCE_NS_TRAJECTORY_H

#include <cmath>

#include <geometry_msgs/Point.h>

#include <reactive_assistance/dist_util.hpp>

namespace reactive_assistance 
{
  // Represents the circular arcs whose centres lie along the robot's y-axis
  class Trajectory
  { 
    public:
      Trajectory(const geometry_msgs::Point& ep)  
                : end_point(ep)
                , radius(computeTrajRadius())
                , tangent_dir(computeTangentDir())
      {} 
      Trajectory(const geometry_msgs::Point& ep, double r)  
                : end_point(ep)
                , radius(r)
                , tangent_dir(computeTangentDir())
      {}
      ~Trajectory() {}
      
      // Robot orientation at point 'p' when tangent to trajectory circle
      inline double getOrientation(const geometry_msgs::Point& p) const
      {
        if (p.y >= 0.0)
        {
          return std::atan2(p.x, ((p.x*p.x)-(p.y*p.y))/(2.0*p.y));
        }
        else
        {
          return -std::atan2(p.x, -((p.x*p.x)-(p.y*p.y))/(2.0*p.y));
        }
      }

      // Return the length of the arc of the trajectory to point 'p'
      inline double getLengthArc(const geometry_msgs::Point& p) const
      {
        if (almostEqual(p.y, 0.0))
        {
          return std::abs(p.x);
        }
        else
        {
          return std::abs(getOrientation(p)*radius);
        }
      }

      // Return the 'closest' point along the trajectory to another workspace point 'p'
      inline void getClosestPoint(const geometry_msgs::Point& p, geometry_msgs::Point& closest) const
      {
        double mag = std::hypot(p.x, p.y-radius);

        closest.x = (p.x/mag) * std::abs(radius);
        closest.y = radius + ((p.y - radius)/mag)*std::abs(radius);
        closest.z = p.z;
      }

      // Getter for the end point
      const geometry_msgs::Point& getGoalPoint() const { return end_point; }
      // Getter for the radius
      double getRadius() const { return radius; }
      // Getter for the tangent direction
      double getDirection() const { return tangent_dir; }

    private:
      // Compute the radius of the trajectory
      inline double computeTrajRadius() const
      {
        return (end_point.x*end_point.x + end_point.y*end_point.y)/(2.0*end_point.y);
      }

      // Compute tangent direction of the trajectory, represents the circular arc
      inline double computeTangentDir() const
      {
        if (end_point.x >= 0)
        {
          return std::atan(1.0/radius);
        }
        else
        {
          return (sgn(end_point.y)*M_PI - std::atan(1.0/radius));
        }
      }

      // End point of goal trajectory
      geometry_msgs::Point end_point;

      double radius;
      double tangent_dir;   
  };
} /* namespace reactive_assistance */
     
#endif