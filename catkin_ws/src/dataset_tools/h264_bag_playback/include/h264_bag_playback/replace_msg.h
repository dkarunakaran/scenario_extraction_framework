#ifndef REPLACE_MSG_H
#define REPLACE_MSG_H

//#include <iostream>

//#include <pluginlib/class_list_macros.h>
//#include <nodelet/nodelet.h>

#include <ros/ros.h>

//#include <sensor_msgs/Imu.h>
//#include <nav_msgs/Odometry.h>

//#include <stdio.h>
//#include <math.h> //fabs

#include <rosbag/view.h>

//#include <tf2_msgs/TFMessage.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
//#include <tf2/buffer_core.h>
//#include <tf2_ros/buffer.h>
//#include <tf2/LinearMath/Transform.h>
//#include <tf2_ros/transform_broadcaster.h>

#include "h264_bag_playback.hpp"
//#include "bag_container.hpp"
//#include "video.hpp"


namespace dataset_toolkit
{


class replace_msg : public h264_bag_playback  {

public:
    replace_msg();


protected:
    void MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message);
    void CameraInfoPublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message,
                             const sensor_msgs::CameraInfoConstPtr &scaled_info_msg);
    void StaticTfPublisher(rosbag::Bag &bag, bool do_publish=true);


    rosbag::Bag out_bag;

};
}

#endif // REPLACE_MSG_H
