#ifndef REACTIVE_ASSISTANCE_NS_DIST_UTIL_H
#define REACTIVE_ASSISTANCE_NS_DIST_UTIL_H

#include <cmath>

#include <geometry_msgs/Point.h>

namespace reactive_assistance 
{
  const double epsilon = 0.0001;
  static const double M_2PI = 2.0*M_PI;

  ///// INLINE FUNCTIONS /////

  // Euclidean distance between Cartesian points a & b
  inline double dist(const geometry_msgs::Point& a, const geometry_msgs::Point& b)
  {
    return std::hypot(a.x-b.x, a.y-b.y);
  }

  // Project scalar into range [-PI, PI)
  inline double proj(double th) 
  {
    if (th >= M_PI || th < -M_PI ){
      th = std::fmod(th, M_2PI);            // in [-2*PI, 2*PI]
      
      if (th < -M_PI) th += M_2PI;          // in [-PI, PI)
      if (th >= M_PI) th -= M_2PI;
    }
    
    return th;
  }
  
  // Project scalar into range [0, 2*PI)
  inline double mod2pi(double th) 
  {
    if (th >= M_2PI || th < 0.0 ){
      th = std::fmod(th, M_2PI);            // in [-2*PI, 2*PI]

      if (th < 0.0) th += M_2PI;            // in [0, 2*PI)
      if (th >= M_2PI) th -= M_2PI;
    }

    return th;
  }

  // Project scalar into range [0, PI)
  inline double modpi(double th) 
  {
    if (th >= M_PI || th < 0.0 ){
      th = std::fmod(th, M_PI);             // in [-PI, PI]

      if (th < 0.0) th += M_PI;             // in [0, PI)
      if (th >= M_PI) th -= M_PI;
    }

    return th;
  }

    // Saturate if necessary
  inline double sat(double x, double lower, double upper)
  {
    if (x <= lower)
      return lower;
    else if (x >= upper)
      return upper;
    else
      return x;
  }

  // Sign function, returns 1 if x > 0.0, -1 if x < 0.0, and 0 otherwise
  inline int sgn(double x) 
  {
    return (x > 0.0) - (x < 0.0);
  }

  // Check if two doubles a & b are almost equal
  inline bool almostEqual(double a, double b)
  {
    return std::abs(a - b) <= epsilon;
  }

  ///// FUNCTION DECLARATIONS /////

  // Transform a point 'p' relative to a frame defined by the angle 'th' and origin point 'org'
  void transformPoint(const geometry_msgs::Point& org, double th, geometry_msgs::Point& p);

  // Check if a 'target' angle is between two other angles (i.e. the interior of the gap)
  // Look at (https://www.xarg.org/2010/06/is-an-angle-between-two-other-angles/) for implementation
  bool isBetweenAngles(double target, double first, double second); 

  // Check if two lines (p1->p2) and (p3->p4) intersect one another
  // Look at Paul Bourke (http://paulbourke.net/geometry/pointlineplane/) for implementation  
  bool lineIntersect(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& p3, const geometry_msgs::Point& p4, geometry_msgs::Point& out);

  // Check if a line defined by p1->p2 intersects with a circle (cx, cy) of radius 'r'
  // Return intersecting point as 'out', if two solutions then take the closest of the intersects
  bool circleIntersect(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& c, double r, geometry_msgs::Point& out);
} /* namespace reactive_assistance */
     
#endif