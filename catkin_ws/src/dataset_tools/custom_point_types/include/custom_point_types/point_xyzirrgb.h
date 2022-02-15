#ifndef POINTXYZIRRGB_CLASS
#define POINTXYZIRRGB_CLASS

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>

// Algorithms we want this type to work with
#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace pcl {

  struct PointXYZIRRGB {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring;
    float rgb;


    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;

}  // end namespace pcl


POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRRGB,
      (float, x, x)
      (float, y, y)
      (float, z, z)
      (float, intensity, intensity)
      (std::uint16_t, ring, ring)
      (float, rgb, rgb))

typedef pcl::PointCloud<pcl::PointXYZIRRGB> PointCloudXYZIRRGB;

#endif
