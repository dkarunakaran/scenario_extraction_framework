//
// Created by stew on 22/05/20.
//

#ifndef DATASET_TOOLS_BAG_CONTAINER_HPP
#define DATASET_TOOLS_BAG_CONTAINER_HPP

#include <set>
#include <string>

#include <ros/ros.h>
#include <rosbag/view.h>

namespace dataset_toolkit {


  class BagContainer {
  public:

    ~BagContainer() {
      bag.close();
    }

    bool Open(std::string file_name) {

      bag_file_name = file_name;

      bag.open(file_name);

      if (!bag.isOpen()) {
        ROS_INFO_STREAM("Could not OPEN bagfile " << file_name);
        return false;
      }

      rosbag::View connections_view(bag);
      for (const rosbag::ConnectionInfo *info: connections_view.getConnections()) {
        topics.insert(info->topic);
      }
      return true;
    }

    rosbag::Bag bag;
    std::shared_ptr <rosbag::View> view;
    rosbag::View::iterator iter;
    std::string bag_file_name;
    std::set <std::string> topics;
  };


}

#endif //DATASET_TOOLS_BAG_CONTAINER_HPP
