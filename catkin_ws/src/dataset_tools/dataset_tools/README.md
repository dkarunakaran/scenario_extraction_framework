The dataset tools contains a number of classes that are commonly used throughout
nodes that work with the ACFR dataset.

## run_pipeline
This code provides a function to publish a message, then block until receiving
a specific response. The code allows waiting for multiple messages of different
types. An example of an implementation using this base class is 
shown below:

```
<header file>
#include "run_pipeline.hpp"

#include "point_xyzir.h"
#include "point_xyzirc.h"

class ICPMatcherPipeline : public RunPipeline {

public:
  ICPMatcherPipeline();
  ~ICPMatcherPipeline() {}

  void receive_message(const pcl::PointCloud<pcl::PointXYZIRC>::Ptr& poles_pointcloud,
                       const pcl::PointCloud<pcl::PointXYZIRC>::Ptr& corners_pointcloud);
private:
  PipelineInput<pcl::PointCloud<pcl::PointXYZIRC>> input_poles, input_corners;
  PipelineOutput<nav_msgs::Odometry> output_pose;

};

<cpp file>
#include "icp_matcher_pipeline.hpp"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>


ICPMatcherPipeline::ICPMatcherPipeline() {

  input_poles.Initialise("/velodyne/front/pole_stacker/average");
  input_corners.Initialise("/velodyne/front/corner_stacker/average");

  output_pose.Initialise("/icp/icp_matcher/odom_corrected", pipes_out);
}


void
ICPMatcherPipeline::receive_message(const pcl::PointCloud<pcl::PointXYZIRC>::Ptr& poles_pointcloud,
                                    const pcl::PointCloud<pcl::PointXYZIRC>::Ptr& corners_pointcloud) {

  ResetMessageFlags();

  input_poles.PublishMessage(poles_pointcloud);
  input_corners.PublishMessage(corners_pointcloud);

  ROS_INFO_STREAM("wait for ICP matcher");

  if (WaitForMessages()) {
    ROS_INFO_STREAM("ICP Matcher received response " << output_pose.last_message->pose.pose.position.x << ", "
                                         << output_pose.last_message->pose.pose.position.y);
  }
  else {
    ROS_INFO_STREAM("ICP Matcher No messages received");
  }
}

```