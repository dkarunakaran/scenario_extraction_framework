#ifndef H264_BAG_PLAYBACK_HEADER
#define H264_BAG_PLAYBACK_HEADER

#include <iostream>
#include <algorithm>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <stdio.h>
#include <math.h> //fabs

#include <rosbag/view.h>

#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>

#include <dataset_msgs/DatasetEvent.h>

#include <boost/algorithm/string.hpp>

#include "corrected_imu_playback.hpp"
#include "bag_static_transform_publisher.hpp"
#include "bag_container.hpp"
#include "video.hpp"


namespace dataset_toolkit {

  class BagWriter {
  public:
    BagWriter() {}

    BagWriter(std::string &filename_prefix) {
      prefix = filename_prefix;
    }

    ~BagWriter() {
      this->CloseBags();
    }

    void SetPrefix(std::string &filename_prefix){
      prefix = filename_prefix;
    }

    void CloseBags() {
      for (auto bag: output_bags)
        bag.second->close();

      for (auto video: output_videos)
        video.second.release();

      output_bags.clear();
    }

    template <typename MsgType>
    bool WriteMessage(std::string &bag_name, std::string &topic_name, ros::Time &msg_time, MsgType msg, std::string additional_folder_name = "") {

      auto bag_instance = output_bags.find(bag_name);

      if (bag_instance == output_bags.end()) {

        std::shared_ptr<rosbag::Bag> new_bag = std::make_shared<rosbag::Bag>();

        output_bags[bag_name] = new_bag;

        std::string modified_prefix = prefix;

        // inject the additional folder name to the prefix
        if (additional_folder_name != "") {
          std::size_t found = prefix.find_last_of('/');
          if (found!=std::string::npos) {
            modified_prefix = prefix.substr(0,found+1) + additional_folder_name + prefix.substr(found);
          }
        }

        std::string additional_bag_name;
        if (bag_name != "") {
          additional_bag_name = modified_prefix + "." + bag_name + ".bag";
        }
        else {
          additional_bag_name = modified_prefix + ".bag";
        }
        ROS_INFO_STREAM("Saving data to additional bagfile named " << additional_bag_name);

        new_bag->open(additional_bag_name, rosbag::bagmode::Write);
      }

      output_bags[bag_name]->write(topic_name, msg_time, msg);
      return true;
    }

    bool WriteToVideo(std::string &bag_name, std::string &topic_name, ros::Time &msg_time, sensor_msgs::Image::Ptr image,
                      sensor_msgs::CameraInfoPtr camera_info_msg = NULL, std::string additional_folder_name = "") {

      auto video_instance = output_videos.find(topic_name);

      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
      }

      if (video_instance == output_videos.end()) {


        std::vector<std::string> topic_parts;
        boost::split(topic_parts, topic_name, boost::is_any_of("/"));

        //_gmsl_A2_image_color_colored_pointcloud
        std::string modified_prefix = prefix;
        if (additional_folder_name != "") {
          // inject the additional folder name to the prefix

          std::size_t found = prefix.find_last_of('/');
          if (found!=std::string::npos) {
            modified_prefix = prefix.substr(0,found+1) + additional_folder_name + prefix.substr(found);
          }
        }

        std::string video_file_name = modified_prefix + "-" + topic_parts[2] + ".mp4";

        //std::string renamed_topic = topic_name;
        //std::replace(renamed_topic.begin(), renamed_topic.end(), '/', '_');
        ROS_INFO_STREAM("Opening video to write new data with name " << video_file_name);
        //return false;

        output_videos[topic_name] = cv::VideoWriter();
        message_count[topic_name] = 1;
        //output_videos[topic_name].open(video_file_name, cv::VideoWriter::fourcc('H','2','6','4'), 30, cv::Size(cv_ptr->image.cols,cv_ptr->image.rows));
        output_videos[topic_name].open(video_file_name, cv::VideoWriter::fourcc('X','V','I','D'), 30, cv::Size(cv_ptr->image.cols,cv_ptr->image.rows));

        if(!output_videos[topic_name].isOpened()) { // check if we succeeded
          ROS_INFO_STREAM("could not open video file: " << video_file_name);
          return false;
        }

        output_videos[topic_name].write(cv_ptr->image);
      }
      else {
        message_count[topic_name] += 1;
        output_videos[topic_name].write(cv_ptr->image);
      }

      gmsl_frame_msg::FrameInfo::Ptr frame_info = boost::make_shared<gmsl_frame_msg::FrameInfo>();
      frame_info->header.stamp = msg_time;
      frame_info->ros_timestamp = msg_time;
      frame_info->frame_counter = message_count[topic_name];
      frame_info->global_counter = message_count[topic_name];
      frame_info->camera_timestamp = 0;

//      ros::Time nvidia_timestamp = ros::Time((frame_info_msg->camera_timestamp) / 1000000.0, ((frame_info_msg->camera_timestamp) % 1000000) * 1000.0);

      std::string frame_info_topic = topic_name + "/frame_info";
      std::string camera_info_topic = topic_name + "/camera_info";

      this->WriteMessage(bag_name, frame_info_topic, msg_time, frame_info);

      if (camera_info_msg) {
        this->WriteMessage(bag_name, camera_info_topic, msg_time, camera_info_msg);
      }

      return true;
    }


//    sensor_msgs::Image::Ptr

    //write compressed image
    /*
void OpenCVConnector::PublishJpeg(uint8_t* image_compressed, uint32_t image_compressed_size) {
sensor_msgs::CompressedImage c_img_msg;

c_img_msg.data.resize( image_compressed_size );
memcpy(&c_img_msg.data[0], image_compressed, image_compressed_size);

std_msgs::Header header; // empty header
c_img_msg.header = header;
//c_img_msg.header.seq = counter; // user defined counter
c_img_msg.header.stamp = ros::Time::now(); // time

c_img_msg.format = "jpeg";

pub_comp.publish(  c_img_msg  );

 camera_info.roi.do_rectify=true;
pubCamInfo.publish(  camera_info ); */

    std::string prefix;

  private:
    std::map<std::string, std::shared_ptr<rosbag::Bag>> output_bags;

    std::map<std::string, cv::VideoWriter> output_videos;
    std::map<std::string, uint32_t> message_count;

  };

  class h264_bag_playback : public nodelet::Nodelet {
  public:

    h264_bag_playback();

    void init_playback();

    void bypass_init() {
      this->onInit();
    }

    ros::Timer timer;

    void timerCallback(const ros::TimerEvent &event);

    // convenience function to open, then block while reading the bag one message at a time
    void ReadFromBag();

    // jump forwards or backwards to a specific time
    void SeekTime(ros::Time seek_time);

    std::shared_ptr <tf2_ros::Buffer> transformer_;

    ros::NodeHandle private_nh;
    ros::NodeHandle public_nh;

    void OpenBags();

    void CloseBags();

    bool ReadNextPacket();

    std::shared_ptr <CorrectedImuPlayback> imu_view;
    std::shared_ptr <rosbag::View> tf_view;

    rosbag::View::iterator tf_iter;
    ros::Time last_tf_time;

    // used to determine when the message time from latest read message
    ros::Time last_packet_time;

    // the requested bag start/end time (based on the params for percentage or specific start/end times)
    ros::Time requested_start_time;
    ros::Time requested_end_time;

    std::list <std::shared_ptr<BagContainer>> bags;

    // A video object for each video file being read
    std::map <std::string, Video> videos;

    // determine whether the playback is real-time (with optional scaling) or as fast as possible
    bool limit_playback_speed;
    double scale_playback_speed = 1.0;

    // times used to synchronise the playback
    ros::Time time_sync_real;
    ros::Time time_sync_playback;
    ros::Time time_sync_latest;


    std::map <std::string, std::list<dataset_msgs::DatasetEvent::ConstPtr>> dataset_events;

    BagWriter bag_writer;

    bool play_all_cameras;

  protected:

    virtual void onInit();

    void AdvertiseTopics(std::shared_ptr <rosbag::View> view);


    // A publisher for each topic in the bag
    std::map <std::string, ros::Publisher> publishers;


    // These are hard coded for the time being to fit the ACFR campus dataset
    std::map <std::string, std::string> frame_id_dict = {{"A0", "gmsl_centre_link"},
                                                         {"A1", "gmsl_left_link"},
                                                         {"A2", "gmsl_right_link"},
                                                         {"A3", "gmsl_back_link"},
                                                         {"B0", "gmsl_left_side_link"},
                                                         {"B1", "gmsl_right_side_link"}};


    virtual void MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message);

    virtual void ImagePublisher(image_transport::Publisher &publisher, const sensor_msgs::ImageConstPtr &message);

    virtual void CameraInfoPublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message,
                                     const sensor_msgs::CameraInfoConstPtr &scaled_info_msg);


    image_transport::ImageTransport image_transport;

    std::string bag_file_name;
    int scaled_width;
    int scaled_height;

    ros::Duration time_offset_;

    // the start and end times from the main bag (before user params are applied)
    ros::Time bag_start_time, bag_end_time;

    // parameters for selecting part of the dataset
    ros::Time playback_start, playback_end;
    ros::Duration playback_duration;


    bool camera_time_bias_flag = false;
    ros::Duration camera_time_bias;

    // determine whether to calculate and publish the horizon transform
    bool horizonInBuffer;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    // store the total number of messages
    uint32_t total_message_count;


  public:
    BagStaticTransformBroadcaster tf_static;

  };

}


#endif
