#ifndef REACTIVE_ASSISTANCE_NS_REACT_ASS_TYPES_H
#define REACTIVE_ASSISTANCE_NS_REACT_ASS_TYPES_H

#include <boost/shared_ptr.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <reactive_assistance/gap.hpp>
#include <reactive_assistance/trajectory.hpp>

namespace reactive_assistance 
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef PointCloud::Ptr PointCloudPtr; 
  typedef boost::shared_ptr<Gap> GapPtr;
  typedef boost::shared_ptr<Trajectory> TrajPtr;
} /* namespace reactive_assistance */
     
#endif