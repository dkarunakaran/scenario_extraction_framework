#include "icp_matcher_pipeline.hpp"
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

ICPMatcherPipeline::ICPMatcherPipeline(): datum_x_(0.), datum_y_(0.) {

  input_poles.Initialise("/velodyne/front/pole_stacker/average");
  input_corners.Initialise("/velodyne/front/corner_stacker/average");

  output_pose.Initialise("/localiser/icp_matcher/odom_corrected", pipes_out);
}


void
ICPMatcherPipeline::receive_message(const pcl::PointCloud<pcl::PointXYZIRC>::Ptr& poles_pointcloud,
                                    const pcl::PointCloud<pcl::PointXYZIRC>::Ptr& corners_pointcloud) {

  ResetMessageFlags();

  input_poles.PublishMessage(poles_pointcloud);
  input_corners.PublishMessage(corners_pointcloud);


//  if (datum_x_ == 0. || datum_y_ == 0.) {

//    tf::StampedTransform transform;
//    try {
//      transform_listener.lookupTransform("utm", "map", ros::Time(0), transform);
//      datum_x_ = transform.getOrigin().x();
//      datum_y_ = transform.getOrigin().y();
//      ROS_WARN_STREAM("icp_matcher_pipeline initialised datum: " << datum_x_ << ", " << datum_y_ );
//    }
//    catch (tf::TransformException &ex) {
//      ROS_WARN_STREAM_THROTTLE(1, "icp_matcher_pipeline is looking for datum: " << ex.what());
//    }
//  }


  if (WaitForMessages()) {
//    ROS_INFO_STREAM("ICP Matcher received response " << output_pose.last_message->pose.pose.position.x << ", "
//                                         << output_pose.last_message->pose.pose.position.y);

//    output_pose.last_message->pose.pose.position.x += datum_x_;
//    output_pose.last_message->pose.pose.position.y += datum_y_;

//    if(std::isnan(output_pose.last_message->pose.pose.position.x) ){
//      ROS_INFO_STREAM_THROTTLE(1, "ICP Matcher received response " << std::fixed << output_pose.last_message->pose.pose.position.x << ", "
//                                         << output_pose.last_message->pose.pose.position.y);
//    }else{
//        ROS_INFO_STREAM_THROTTLE(1, "ICP Matcher received response " << std::fixed << output_pose.last_message->pose.pose.position.x << ", "
//                                             << output_pose.last_message->pose.pose.position.y );
//    }

    if (publish_pose) {
      publish_pose(output_pose.last_message);
    }
  }
  else {
    ROS_INFO_STREAM("ICP Matcher No messages received");
    output_pose.last_message = NULL;
  }
}
