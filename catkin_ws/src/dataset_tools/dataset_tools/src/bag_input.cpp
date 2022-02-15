#include "bag_input.hpp"


#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_datatypes.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>



BagInput::BagInput() {

  bag = std::make_shared<rosbag::Bag>();

}

BagInput::~BagInput() {
  bag->close();
}

/*
void BagOutput::publish_odom(nav_msgs::Odometry &msg, std::string topic_name) {
  if (bag.isOpen())
    bag.write(topic_name, msg.header.stamp, msg);
}


void BagOutput::publish_fix(sensor_msgs::NavSatFix &msg, std::string topic_name) {
  if (bag.isOpen())
    bag.write(topic_name, msg.header.stamp, msg);
}
*/

void BagInput::ReadBag(std::string bag_file) {

  //PointCloudFeaturesPipeline run_pipeline();

  bag->open(bag_file);
  for(rosbag::MessageInstance const m: rosbag::View(*bag))
  {
    if (publish_imu_update) {
      auto msg = m.instantiate<sensor_msgs::Imu>();
      if (msg != NULL && imu_topics.count(m.getTopic()) != 0)
        publish_imu_update(msg);
    }

    if (publish_fix_update) {
      auto msg = m.instantiate<sensor_msgs::NavSatFix>();
      if (msg && fix_update_topics.count(m.getTopic()) != 0) {
        publish_fix_update(msg);
      }
    }

    if (publish_pointcloud_update) {
      auto msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (msg && pointcloud_topics.count(m.getTopic()) != 0)
        publish_pointcloud_update(msg);
    }

    if (publish_speed_update) {
      auto msg = m.instantiate<nav_msgs::Odometry>();
      if (msg && odom_speed_topics.count(m.getTopic()) != 0)
        publish_speed_update(msg);
    }

    if (publish_odom_update) {
      auto msg = m.instantiate<nav_msgs::Odometry>();
      if (msg && odom_update_topics.count(m.getTopic()) != 0)
        publish_odom_update(msg);
    }

    if (!ros::ok())
      break;

    ros::spinOnce();
  }

  bag->close();

}

