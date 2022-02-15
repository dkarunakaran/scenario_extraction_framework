#ifndef POINTXYZIR_CLASS
#define POINTXYZIR_CLASS

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>

// Algorithms we want this type to work with
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/impl/crop_box.hpp>


namespace pcl {

  struct PointXYZIR {
    PCL_ADD_POINT4D                  // Macro quad-word XYZ
    float intensity;             // Laser intensity
    uint16_t ring;                   // Laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Ensure proper alignment
  } EIGEN_ALIGN16;

}  // end namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring))

typedef pcl::PointCloud<pcl::PointXYZIR> PointCloudXYZIR;

#endif
