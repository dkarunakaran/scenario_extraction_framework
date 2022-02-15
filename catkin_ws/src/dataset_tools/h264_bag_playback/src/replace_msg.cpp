#include "replace_msg.h"
#include <rosbag/bag.h>
//#include <rosbag/player.h>
//#include <rosbag/structures.h>
#include <rosbag/message_instance.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace dataset_toolkit
{
replace_msg::replace_msg()
{

    out_bag.open("replace_msg.bag", rosbag::bagmode::Write);


//    bag.close();
}

/**
 * @brief h264_bag_playback::StaticTfPublisher extract and publish /tf_static and store in member tf buffer transformer_
 * only reads in the first 10 /tf_static messages and discard all msgs after 10, to prevent a node spaming /tf_static msgs.
 * @param bag bag that contains static tf
 * @param do_publish if want to publish /tf_static defaut to true.
 */
void replace_msg::StaticTfPublisher(rosbag::Bag &bag, bool do_publish) {

  static tf2_ros::StaticTransformBroadcaster static_broadcaster;

  std::vector<std::string> topics;

  topics.push_back(std::string("tf_static"));
  topics.push_back(std::string("/tf_static"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  int n_static_tf = 0;
  for(rosbag::MessageInstance const &m: view) {
    const auto tf = m.instantiate<tf2_msgs::TFMessage>();
    tf2_msgs::TFMessage::Ptr tf_corrected = boost::make_shared<tf2_msgs::TFMessage>();
    *tf_corrected = *tf;
    ROS_ERROR_STREAM("tf_corrected->transforms.size() "<<tf_corrected->transforms.size());
    tf_corrected->transforms.clear();

    if (do_publish) {
      static_broadcaster.sendTransform(tf->transforms);
    }

    for (auto &transform: tf->transforms) {

      if(transform.child_frame_id == "velodyne_front_link"){
          // replace base->velodyne for testing

          tf2::Transform odom_tf;
          tf2::fromMsg(transform.transform, odom_tf);
          double r,p,yaw;
          odom_tf.getBasis().getRPY(r,p,yaw);
          ROS_ERROR_STREAM("original velodyne tf yaw "<<yaw << " pitch " << p << " roll " << r);

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "base_link" ;
        transformStamped.child_frame_id = "velodyne_front_link";

        transformStamped.transform.translation.x = transform.transform.translation.x;
        transformStamped.transform.translation.y = transform.transform.translation.y;
        transformStamped.transform.translation.z = transform.transform.translation.z;

        float new_roll, new_pitch, new_yaw;
        private_nh.param<float>("new_roll", new_roll, 0.);
        private_nh.param<float>("new_pitch", new_pitch, 0.);
        private_nh.param<float>("new_yaw", new_yaw, 0.);
        ROS_ERROR_STREAM("offseting velodyne tf yaw "<<new_yaw << " pitch " << new_pitch << " roll " << new_roll);

        new_yaw += yaw;
        new_pitch += p;
        new_roll += r;

        tf2::Quaternion quat;
        quat.setRPY(new_roll, new_pitch, new_yaw);

        transformStamped.transform.rotation.x = quat.x();
        transformStamped.transform.rotation.y = quat.y();
        transformStamped.transform.rotation.z = quat.z();
        transformStamped.transform.rotation.w = quat.w();

        static_broadcaster.sendTransform(transformStamped);
        transformer_->setTransform(transformStamped, "zio", true);
        tf_corrected->transforms.push_back(transformStamped);

        tf2::Transform odom_tf_2;
        tf2::fromMsg(transformStamped.transform, odom_tf_2);
        odom_tf_2.getBasis().getRPY(r,p,yaw);
        ROS_ERROR_STREAM("velodyne tf now reads yaw "<<yaw << " pitch " << p << " roll " << r);

      }else if(transform.child_frame_id == "utm"){
        continue;
      }else{
        transformer_->setTransform(transform, "zio", true);
        tf_corrected->transforms.push_back(transform);
      }

    }

    std::string const& topic = m.getTopic();
    ros::Time const& header_time = m.getTime();
    out_bag.write(topic, header_time, tf_corrected);

    n_static_tf++;
    if(n_static_tf>10)
        break;

  }
    ROS_INFO_STREAM("Loaded static TF tree data");
}


void
replace_msg::CameraInfoPublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message,
                                 const sensor_msgs::CameraInfoConstPtr &scaled_info_msg) {
    std::string const& topic = message.getTopic();
    ros::Time const& header_time = message.getTime();
    out_bag.write(topic, header_time, scaled_info_msg);
}


void
replace_msg::MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message) {
    std::string const& topic = message.getTopic();
    ros::Time const& header_time = message.getTime();

    if (topic == "vn100/imu" || topic == "/vn100/imu") {

        auto msg = message.instantiate<sensor_msgs::Imu>();
        if (msg) {
          // compute horizon transforms from imu msg and publish them
          geometry_msgs::TransformStamped baselink, footprint;
          imu2horizontf(msg, baselink, footprint);
//          tf_broadcaster.sendTransform(baselink);
//          tf_broadcaster.sendTransform(footprint);

          tf2_msgs::TFMessage tf_horizon;
          tf_horizon.transforms.push_back(baselink);
          tf_horizon.transforms.push_back(footprint);

          out_bag.write("/tf", header_time, tf_horizon);
        }
    }

    out_bag.write(topic, header_time, message);
}
}

int main(int argc, char **argv) {

  //Initialize Node and handles
  ros::init(argc, argv, "replaceMsg_writeToBag");
  ros::NodeHandle n;

  dataset_toolkit::replace_msg bag_tools;
  //bag_tools.bypass_init();
  bag_tools.init_playback();
  bag_tools.ReadFromBag();

  return 0;
}
