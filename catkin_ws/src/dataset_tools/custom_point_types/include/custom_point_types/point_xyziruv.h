#ifndef POINTXYZIRUV_CLASS
#define POINTXYZIRUV_CLASS

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>

// Algorithms we want this type to work with
#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace pcl
{

  struct PointXYZIRUV {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring;
    float U;
    float V;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;

}  // end namespace pcl


POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRUV,
      (float, x, x)
      (float, y, y)
      (float, z, z)
      (float, intensity, intensity)
      (std::uint16_t, ring, ring)
      (float, U, U)
      (float, V, V))


typedef pcl::PointCloud<pcl::PointXYZIRUV> PointCloudXYZIRUV;

#endif
