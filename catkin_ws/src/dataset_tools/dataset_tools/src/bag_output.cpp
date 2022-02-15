#include "bag_output.hpp"

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>



#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>





BagOutput::BagOutput() {

  bag = std::make_shared<rosbag::Bag>();

}

BagOutput::~BagOutput() {
  bag->close();
}


void BagOutput::publish_stats(dataset_msgs::LocaliserStats &msg, std::string topic_name){
  if (bag->isOpen())
    bag->write(topic_name, msg.header.stamp, msg);
}


void BagOutput::publish_odom(nav_msgs::Odometry &msg, std::string topic_name) {
  if (bag->isOpen())
    bag->write(topic_name, msg.header.stamp, msg);
}


void BagOutput::publish_fix(sensor_msgs::NavSatFix &msg, std::string topic_name) {
  if (bag->isOpen())
    bag->write(topic_name, msg.header.stamp, msg);
}


void BagOutput::publish_tf(tf::StampedTransform &msg, std::string topic_name) {
  if (bag->isOpen()) {

    tf2_msgs::TFMessage tf_pub_message;
    geometry_msgs::TransformStamped geom_tf;
    geom_tf.transform.rotation.x = msg.getRotation()[0];
    geom_tf.transform.rotation.y = msg.getRotation()[1];
    geom_tf.transform.rotation.z = msg.getRotation()[2];
    geom_tf.transform.rotation.w = msg.getRotation()[3];
    geom_tf.transform.translation.x = msg.getOrigin()[0];
    geom_tf.transform.translation.y = msg.getOrigin()[1];
    geom_tf.transform.translation.z = msg.getOrigin()[2];
    geom_tf.header.stamp = msg.stamp_;
    geom_tf.header.frame_id = msg.frame_id_;
    geom_tf.child_frame_id = msg.child_frame_id_;
    tf_pub_message.transforms.push_back(geom_tf);

    bag->write(topic_name, msg.stamp_, tf_pub_message);
  }
}



void BagOutput::Initialise(std::string bag_file) {
  bag->open(bag_file, rosbag::bagmode::Write);
}
