#pragma once

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>

// Algorithms we want this type to work with
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>
#include <pcl/common/impl/centroid.hpp>

namespace pcl
{
struct PointXYZIRC
{
  PCL_ADD_POINT4D                  // Macro quad-word XYZ
      float intensity;             // Laser intensity
  uint16_t ring;                   // Laser ring number
  uint16_t cluster;                // Cluster number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Ensure proper alignment
} EIGEN_ALIGN16;

}  // end namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRC, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                                   uint16_t, ring, ring)(uint16_t, cluster, cluster))

typedef pcl::PointCloud<pcl::PointXYZIRC> PointCloudXYZIRC;
