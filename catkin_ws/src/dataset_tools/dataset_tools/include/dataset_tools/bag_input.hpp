#ifndef bag_input_h
#define bag_input_h

#include <ros/ros.h>

// include messages to write to bag file
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>



namespace rosbag {
  class Bag;
}

/*!
 * \brief Class to publish the ros messages
 *
 */

class BagInput {
public:
  BagInput();
  ~BagInput();

  void ReadBag(std::string bag_file);

  std::function<void(const nav_msgs::Odometry::ConstPtr&)> publish_odom_update;
  std::function<void(const sensor_msgs::NavSatFix::ConstPtr&)> publish_fix_update;
  std::function<void(const nav_msgs::Odometry::ConstPtr&)> publish_speed_update;
  std::function<void(const sensor_msgs::Imu::ConstPtr&)> publish_imu_update;
  std::function<void(const sensor_msgs::PointCloud2::ConstPtr&)> publish_pointcloud_update;

  std::set<std::string> odom_update_topics;
  std::set<std::string> fix_update_topics;
  std::set<std::string> odom_speed_topics;
  std::set<std::string> pointcloud_topics;
  std::set<std::string> imu_topics;

  std::shared_ptr<rosbag::Bag> bag;


};



#endif
