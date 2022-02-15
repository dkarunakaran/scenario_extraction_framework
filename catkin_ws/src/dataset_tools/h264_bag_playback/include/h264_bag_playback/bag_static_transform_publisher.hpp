//
// Created by stew on 16/05/21.
//

#ifndef H264_BAG_PLAYBACK_BAGSTATICTRANSFORMPUBLISHER_HPP
#define H264_BAG_PLAYBACK_BAGSTATICTRANSFORMPUBLISHER_HPP

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>


class BagStaticTransformBroadcaster : public tf2_ros::StaticTransformBroadcaster {


public:

/**
 * @brief h264_bag_playback::StaticTfPublisher extract and publish /tf_static and store in member tf buffer transformer_
 * only reads in the first 10 /tf_static messages and discard all msgs after 10, to prevent a node spaming /tf_static msgs.
 * @param bag bag that contains static tf
 * @param do_publish if want to publish /tf_static defaut to true.
 */
  void StaticTfPublisher(rosbag::Bag &bag, bool do_publish, std::shared_ptr<tf2_ros::Buffer> &transformer_) {

    //static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    ros::NodeHandle private_nh("~");
    std::vector <std::string> topics;

    topics.push_back(std::string("tf_static"));
    topics.push_back(std::string("/tf_static"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    int n_static_tf = 0;
    for (rosbag::MessageInstance const &m: view) {
      const auto tf = m.instantiate<tf2_msgs::TFMessage>();
      static_transforms.push_back(tf);

      for (const auto &transform: tf->transforms) {

        if (transform.child_frame_id == "velodyne_front_link") {
          // replace base->velodyne for testing

          tf2::Transform odom_tf;
          tf2::fromMsg(transform.transform, odom_tf);
          double r, p, yaw;
          odom_tf.getBasis().getRPY(r, p, yaw);
          ROS_ERROR_STREAM("original velodyne tf yaw " << yaw << " pitch " << p << " roll " << r);

          geometry_msgs::TransformStamped transformStamped;
          transformStamped.header.stamp = ros::Time::now();
          transformStamped.header.frame_id = "base_link";
          transformStamped.child_frame_id = "velodyne_front_link";

          transformStamped.transform.translation.x = transform.transform.translation.x;
          transformStamped.transform.translation.y = transform.transform.translation.y;
          transformStamped.transform.translation.z = transform.transform.translation.z;

          float new_roll, new_pitch, new_yaw;
          private_nh.param<float>("new_roll", new_roll, 0.);
          private_nh.param<float>("new_pitch", new_pitch, 0.);
          private_nh.param<float>("new_yaw", new_yaw, 0.);
          ROS_ERROR_STREAM("offseting velodyne tf yaw " << new_yaw << " pitch " << new_pitch << " roll " << new_roll);

          new_yaw += yaw;
          new_pitch += p;
          new_roll += r;

          tf2::Quaternion quat;
          quat.setRPY(new_roll, new_pitch, new_yaw);

          transformStamped.transform.rotation.x = quat.x();
          transformStamped.transform.rotation.y = quat.y();
          transformStamped.transform.rotation.z = quat.z();
          transformStamped.transform.rotation.w = quat.w();

          this->sendTransform(transformStamped);
          transformer_->setTransform(transformStamped, "zio", true);

          tf2::Transform odom_tf_2;
          tf2::fromMsg(transformStamped.transform, odom_tf_2);
          odom_tf_2.getBasis().getRPY(r, p, yaw);
          ROS_ERROR_STREAM("velodyne tf now reads yaw " << yaw << " pitch " << p << " roll " << r);

        } else if (transform.child_frame_id == "utm") {
          continue;
        } else {
          transformer_->setTransform(transform, "zio", true);
        }

        if (do_publish) {
          //this->sendTransform(tf->transforms);
          this->sendTransform(transform);
        }
      }
      n_static_tf++;
      if (n_static_tf > 10)
        break;

    }
    ROS_INFO_STREAM("Loaded static TF tree data");
  }


  std::list<tf2_msgs::TFMessageConstPtr> static_transforms;

};

#endif //H264_BAG_PLAYBACK_BAGSTATICTRANSFORMPUBLISHER_HPP
