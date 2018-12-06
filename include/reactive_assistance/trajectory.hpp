#ifndef REACTIVE_ASSISTANCE_NS_TRAJECTORY_H
#define REACTIVE_ASSISTANCE_NS_TRAJECTORY_H

#include <cmath>

#include <geometry_msgs/Point.h>

#include <reactive_assistance/dist_util.hpp>

namespace reactive_assistance 
{
  class Trajectory
  { 
    public:
      Trajectory(const geometry_msgs::Point& ep);
      Trajectory(const geometry_msgs::Point& ep, double r); 
      ~Trajectory() {}
      
      ///// INLINE FUNCTIONS /////

      // Robot orientation at point 'p' when tangent to trajectory circle
      inline double getOrientation(const geometry_msgs::Point& p) const
      {
        if (p.y >= 0.0)
          return std::atan2(p.x, ((p.x*p.x)-(p.y*p.y))/(2.0*p.y));
        else
          return -std::atan2(p.x, -((p.x*p.x)-(p.y*p.y))/(2.0*p.y));
      }

      // Return the length of the arc of the trajectory to point 'p'
      inline double getLengthArc(const geometry_msgs::Point& p) const
      {
        if (almostEqual(p.y, 0.0))
          return std::abs(p.x);
        else
          return std::abs(getOrientation(p)*radius);
      }

      // Return the 'closest' point along the trajectory to another workspace point 'p'
      inline void getClosestPoint(const geometry_msgs::Point& p, geometry_msgs::Point& closest) const
      {
        double mag = std::hypot(p.x, p.y-radius);

        closest.x = (p.x/mag) * std::abs(radius);
        closest.y = radius + ((p.y - radius)/mag)*std::abs(radius);
        closest.z = p.z;
      }

      ///// GETTERS /////

      // Getter for the trajectory end point
      const geometry_msgs::Point& getGoalPoint() const { return end_point; }
      // Getter for the trajectory radius
      double getRadius() const { return radius; }
      // Getter for the trajectory tangent direction
      double getDirection() const { return tangent_dir; }

    private:
      // Compute the radius of the trajectory
      double computeTrajRadius() const;
      // Compute tangent direction of the trajectory
      double computeTangentDir() const;

      // End point of goal trajectory
      geometry_msgs::Point end_point;

      double radius;
      double tangent_dir;   
  };
} /* namespace reactive_assistance */
     
#endif