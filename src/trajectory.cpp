// All the other necessary headers included in the class declaration files
#include <reactive_assistance/trajectory.hpp>

//==============================================================================
// TRAJECTORY DEFINITIONS
//==============================================================================
namespace reactive_assistance 
{
  //==============================================================================
  // PUBLIC TRAJECTORY METHODS
  //==============================================================================
  Trajectory::Trajectory(const geometry_msgs::Point& ep)  
                          : end_point(ep), 
                            radius(computeTrajRadius()),
                            tangent_dir(computeTangentDir())
                            {} 

  Trajectory::Trajectory(const geometry_msgs::Point& ep, double r)  
                          : end_point(ep), 
                            radius(r),
                            tangent_dir(computeTangentDir())
                            {}

  //==============================================================================
  // PRIVATE TRAJECTORY METHODS (Utilities)
  //============================================================================== 
  // Compute the radius of the trajectory
  double Trajectory::computeTrajRadius() const
  {
    return (end_point.x*end_point.x + end_point.y*end_point.y)/(2.0*end_point.y);
  }

  // Compute tangent direction of the trajectory, represents the circular arc
  double Trajectory::computeTangentDir() const
  {
    if (end_point.x >= 0)
      return std::atan(1.0/radius);
    else
      return (sgn(end_point.y)*M_PI - std::atan(1.0/radius));
  }
} /* namespace reactive_assistance */
