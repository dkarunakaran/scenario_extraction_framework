//
// Created by stew on 27/08/19.
//

#ifndef LOCALISER_ICP_MATCHER_PIPELINE_HPP
#define LOCALISER_ICP_MATCHER_PIPELINE_HPP

#include "run_pipeline.hpp"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <custom_point_types/point_xyzir.h>
#include <custom_point_types/point_xyzirc.h>

class ICPMatcherPipeline : public RunPipeline {

public:
  ICPMatcherPipeline();
  ~ICPMatcherPipeline() {}

  void receive_message(const pcl::PointCloud<pcl::PointXYZIRC>::Ptr& poles_pointcloud,
                       const pcl::PointCloud<pcl::PointXYZIRC>::Ptr& corners_pointcloud);

  std::function<void(const nav_msgs::Odometry::Ptr&)> publish_pose;

  PipelineOutput<nav_msgs::Odometry> output_pose;

private:
  PipelineInput<pcl::PointCloud<pcl::PointXYZIRC>> input_poles, input_corners;

  double datum_x_;
  double datum_y_;

  tf::TransformListener transform_listener;
};

#endif //LOCALISER_POINT_CLOUD_FEATURES_PIPELINE_HPP
