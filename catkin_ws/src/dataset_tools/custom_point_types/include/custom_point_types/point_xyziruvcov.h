#ifndef POINTXYZIRUVCOV_CLASS
#define POINTXYZIRUVCOV_CLASS

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>

namespace pcl
{

  struct PointXYZIRUVCOV {
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring;
    float U;
    float V;
    float cov_xx;
    float cov_yy;
    float cov_zz;
    float cov_xy;
    float cov_xz;
    float cov_yz;
    float cov_UU;
    float cov_VV;
    float cov_UV;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;

}  // end namespace pcl


POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRUVCOV,
      (float, x, x)
      (float, y, y)
      (float, z, z)
      (float, intensity, intensity)
      (std::uint16_t, ring, ring)
      (float, U, U)
      (float, V, V)
      (float, cov_xx, cov_xx)
      (float, cov_yy, cov_yy)
      (float, cov_zz, cov_zz)
      (float, cov_xy, cov_xy)
      (float, cov_xz, cov_xz)
      (float, cov_yz, cov_yz)
      (float, cov_UU, cov_UU)
      (float, cov_VV, cov_VV)
      (float, cov_UV, cov_UV))


typedef pcl::PointCloud<pcl::PointXYZIRUVCOV> PointCloudXYZIRUVCOV;

#endif
