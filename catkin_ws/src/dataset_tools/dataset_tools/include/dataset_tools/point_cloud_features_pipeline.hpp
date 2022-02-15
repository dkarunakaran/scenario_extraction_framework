//
// Created by stew on 27/08/19.
//

#ifndef LOCALISER_POINT_CLOUD_FEATURES_PIPELINE_HPP
#define LOCALISER_POINT_CLOUD_FEATURES_PIPELINE_HPP

#include "run_pipeline.hpp"

#include "custom_point_types/point_xyzir.h"
#include "custom_point_types/point_xyzirc.h"

class PointCloudFeaturesPipeline : public RunPipeline {

public:
  PointCloudFeaturesPipeline();
  ~PointCloudFeaturesPipeline() {}

  void receive_message(const sensor_msgs::PointCloud2::ConstPtr& input_pointcloud);

  std::function<void(const pcl::PointCloud<pcl::PointXYZIRC>::Ptr&, const pcl::PointCloud<pcl::PointXYZIRC>::Ptr&)> publish_poles_corners;


  PipelineOutput<pcl::PointCloud<pcl::PointXYZIRC>> output_poles, output_corners;

private:
  PipelineInput<pcl::PointCloud<pcl::PointXYZIR>> input_pointcloud;

};

#endif //LOCALISER_POINT_CLOUD_FEATURES_PIPELINE_HPP
