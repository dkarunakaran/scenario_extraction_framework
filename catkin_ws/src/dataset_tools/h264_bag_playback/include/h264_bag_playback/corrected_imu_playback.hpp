//
// Created by stew on 16/05/21.
//

#ifndef H264_BAG_PLAYBACK_CORRECTED_IMU_PLAYBACK_HPP
#define H264_BAG_PLAYBACK_CORRECTED_IMU_PLAYBACK_HPP

#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rosbag/view.h>


class CorrectedImuPlayback : public rosbag::View {

public:

  CorrectedImuPlayback(rosbag::Bag const &bag,
      boost::function< bool(rosbag::ConnectionInfo const *)>  	query,
      ros::Time const &  	start_time = ros::TIME_MIN,
      ros::Time const &  	end_time = ros::TIME_MAX) :

      rosbag::View(bag, query, start_time, end_time),
      last_imu_time(ros::Time(0.01)){

  }

  void ResetPlayback() {
    imu_iter = this->begin();
    last_imu_time = ros::Time(0.01);
  }

  void CalculateHorizon(std::shared_ptr<tf2_ros::Buffer> &transformer_,
                        ros::Time &current_time/*,
                        geometry_msgs::TransformStamped &baselink,
                        geometry_msgs::TransformStamped &footprint*/) {

    // if horizonInBuffer param is set, calculate base_link to base_link_horizon tf
    // query the bag imu msgs so that transformer buffers horizon tf up to 2s in the future
    while (imu_iter != this->end()
           && last_imu_time < current_time + ros::Duration(2)) {

      auto imu_msg = imu_iter->instantiate<sensor_msgs::Imu>();

      if (imu_msg) {

        geometry_msgs::TransformStamped baselink, footprint;
        CorrectedImuPlayback::imu2horizontf(imu_msg, baselink, footprint);

        transformer_->setTransform(baselink, "zio", false);
        transformer_->setTransform(footprint, "zio", false);

        last_imu_time = imu_msg->header.stamp;
        imu_iter++;
      }
    }
  }

  static void
  imu2horizontf(sensor_msgs::Imu::Ptr &imu_msg, geometry_msgs::TransformStamped &baselink,
                                   geometry_msgs::TransformStamped &footprint) {
    if (imu_msg) {
      // calculate horizon to base tf, and push to tf buffer
      tf2::Transform imu_tf;
      tf2::Quaternion q1(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
      imu_tf.setRotation(q1);
      double roll, pitch, yaw;
      imu_tf.getBasis().getRPY(roll, pitch, yaw);

      tf2::Quaternion q;
      q.setRPY(-roll, -pitch, 0.);

      baselink.transform.rotation.x = footprint.transform.rotation.x = q.x();
      baselink.transform.rotation.y = footprint.transform.rotation.y = q.y();
      baselink.transform.rotation.z = footprint.transform.rotation.z = q.z();
      baselink.transform.rotation.w = footprint.transform.rotation.w = q.w();
      baselink.header.stamp = footprint.header.stamp = imu_msg->header.stamp;
      baselink.header.frame_id = "base_link";
      footprint.header.frame_id = "base_footprint";
      baselink.child_frame_id = "base_link_horizon";
      footprint.child_frame_id = "base_footprint_horizon";
    }
  }

  ros::Time last_imu_time;

  rosbag::View::iterator imu_iter;
};


#endif //H264_BAG_PLAYBACK_CORRECTED_IMU_PLAYBACK_HPP
