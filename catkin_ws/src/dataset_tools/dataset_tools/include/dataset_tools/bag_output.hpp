#ifndef bag_output_h
#define bag_output_h

// include messages to write to bag file
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <dataset_msgs/LocaliserStats.h>

#include <tf/transform_datatypes.h>


namespace rosbag {
  class Bag;
}

/*!
 * \brief Class to publish the ros messages
 *
 */

class BagOutput {
public:
  BagOutput();
  ~BagOutput();

  void Initialise(std::string bag_file);

  void publish_odom(nav_msgs::Odometry &msg, std::string topic_name);
  void publish_fix(sensor_msgs::NavSatFix &msg, std::string topic_name);
  void publish_tf(tf::StampedTransform &msg, std::string topic_name);
  void publish_stats(dataset_msgs::LocaliserStats &msg, std::string topic_name);

  std::shared_ptr<rosbag::Bag> bag;

};




#endif
