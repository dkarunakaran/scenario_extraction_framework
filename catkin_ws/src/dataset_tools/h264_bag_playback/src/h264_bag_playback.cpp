#include "h264_bag_playback.hpp"
#include <glob.h>

#include "boost/date_time/posix_time/posix_time.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

#include <rosbag/bag.h>
#include <rosbag/player.h>
#include <rosbag/structures.h>
#include <rosbag/message_instance.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <gmsl_frame_msg/FrameInfo.h>

#include <tf2_ros/static_transform_broadcaster.h>

#include "corrected_imu_playback.hpp"
#include "helper_functions.hpp"
#include "video.hpp"


// Nodelet problem is https://github.com/ros/ros_comm/issues/1474

// this main function allows the nodelet to be compiled as a node.
//  this has been done because there is a problem in ros melodic where
//  the rosbag library has a conflict with the compression functions
//  of the opencv library causing a run time seg fault.

int main(int argc, char **argv) {

  //Initialize Node and handles
  ros::init(argc, argv, "h264_bag_playback_node");
  ros::NodeHandle n;

  dataset_toolkit::h264_bag_playback bag_tools;

  bag_tools.init_playback();
  bag_tools.ReadFromBag();

  return 0;
}


namespace dataset_toolkit
{


h264_bag_playback::h264_bag_playback() :
        transformer_(std::make_shared<tf2_ros::Buffer>(ros::Duration(200.))),
        horizonInBuffer(false),
        private_nh("~"),
        image_transport(public_nh),
        playback_start(ros::TIME_MIN),
        playback_end(ros::TIME_MAX),
        total_message_count(0),
        play_all_cameras(false){
}


void h264_bag_playback::onInit() {
  timer = public_nh.createTimer(ros::Duration(0.1), &h264_bag_playback::timerCallback, this);
}


void h264_bag_playback::timerCallback(const ros::TimerEvent& event) {
  ReadFromBag();
}


void h264_bag_playback::init_playback() {

    // parameter to scale the size of the images from the h264 playback
    scaled_width = 0;
    scaled_height = 0;
    private_nh.param("output_width", scaled_width, 0);
    private_nh.param("output_height", scaled_height, 0);

    if (scaled_height && scaled_width) {
      ROS_INFO_STREAM("Output images will be scaled to " << scaled_width << "x" << scaled_height);
    }
    else {
      ROS_INFO_STREAM("Output images will NOT be scaled");
    }


    // parameters to scale, or limit the speed of playback to realtime
    private_nh.param("scale_playback_speed", scale_playback_speed, 1.0);
    private_nh.param("limit_playback_speed", limit_playback_speed, true);


    // determine the bag file to playback
    private_nh.getParam("bag_file", bag_file_name);

    if (bag_file_name.empty()) {
      ROS_INFO_STREAM("Bag file name parameter is missing " << bag_file_name);
      return;
    }


    // Attempt to open the bag file
    bags.push_back(std::make_shared<BagContainer>());
    if (!bags.back()->Open(bag_file_name))
      return;


    // determine the file prefixes and initialise each camera
    std::string file_prefix = remove_last_of_string(bag_file_name, ".");
    std::string dataset_name = keep_last_of_string(file_prefix, "/");
    ROS_INFO_STREAM("Reading from bag: " << bag_file_name);
    ROS_INFO_STREAM("Dataset name " << dataset_name);

    std::vector<std::string> file_list;
    get_files_pattern(file_prefix + "*.mp4", file_list);

    bag_writer.SetPrefix(file_prefix);

    std::vector<std::string> extensions_list;
    get_files_pattern(file_prefix + ".*.bag", extensions_list);
    for (auto extension_name: extensions_list) {
      ROS_INFO_STREAM("Loading extension: " << extension_name);

      auto new_bag = std::make_shared<BagContainer>();
      bags.push_back(new_bag);
      if (!new_bag->Open(extension_name))
        return;

      // for all the other bags, if there is a topic with the same name, delete it from the list
      //  the last opened bags have priority
      for (auto bag: bags) {
        // if it is not this bag
        if (bag != new_bag) {
          for (auto topic_name: new_bag->topics){
            if (bag->topics.find(topic_name) != bag->topics.end() && topic_name != "/tf_static") {
              bag->topics.erase(topic_name);
              ROS_INFO_STREAM ("Replacing " << topic_name << " from bag " << bag->bag_file_name << " as it exists in bag " << new_bag->bag_file_name);
            }
          }
        }
      }
    }


    // make a video object for each video file
    for (auto file_name: file_list) {
      // extract the camera name from the filename
      std::string camera_name = keep_last_of_string(remove_last_of_string(file_name, "."), "-");

      Video new_video;
      videos[camera_name] = new_video;
      if (videos[camera_name].InitialiseVideo(camera_name, file_name)){
        ROS_INFO_STREAM("Loading " << file_name << " for camera " << camera_name);
      }
      else {
        ROS_ERROR_STREAM("FAILED to open video file: " << file_name << " for camera " << camera_name);
        return;
      }
    }


    // determine whether to apply a time bias correction
    // (some datasets have an offset between the nvidia computer time and the ROS time)
    std::map<std::string, double> dataset_time_correction;
    private_nh.getParam("dataset_time_correction", dataset_time_correction);

    ros::Duration time_offset(0.0);
    time_offset_ = time_offset;
    if (dataset_time_correction.count(dataset_name)) {
      time_offset_ = ros::Duration(dataset_time_correction.at(dataset_name));
      ROS_INFO_STREAM("Time correction of " << time_offset_.toSec() << " is being applied");
    }
    else {
      ROS_INFO_STREAM("No time correction parameters available for this dataset");
    }


    // assume the "main" bag covers the most important times
    rosbag::View overall_view(bags.front()->bag);

    bag_start_time = overall_view.getBeginTime();
    bag_end_time = overall_view.getEndTime();

    auto bag_duration = (bag_end_time-bag_start_time).toSec();

    ROS_INFO_STREAM("Bag start time " << boost::posix_time::to_iso_extended_string(bag_start_time.toBoost()));
    ROS_INFO_STREAM("Bag end time " << boost::posix_time::to_iso_extended_string(bag_end_time.toBoost()));
    ROS_INFO_STREAM("Bag duration: " << bag_duration << " seconds");

    // determine which part of the playback is requested by the user
    std::string start_time_param_string, end_time_param_string;
    float start_percentage, end_percentage;
    private_nh.getParam("time_start", start_time_param_string);
    private_nh.getParam("time_end", end_time_param_string);
    private_nh.param<float>("percentage_start", start_percentage, 0);
    private_nh.param<float>("percentage_end", end_percentage, 100);

    if (start_percentage == 0 && end_percentage == 100) {
      //ROS_INFO_STREAM("No requests for the playback of a percentage of the dataset");
    }
    else {
      ROS_INFO_STREAM("Requested playback from " << start_percentage << "\% to " << end_percentage << "\%");
    }

    requested_start_time = bag_start_time;
    requested_end_time = bag_end_time;

    try {
      ros::Time test_start_time = ros::Time::fromBoost(boost::posix_time::from_iso_extended_string(start_time_param_string));
      requested_start_time = test_start_time;
      ROS_INFO_STREAM("Requested start time " << start_time_param_string << " is " << requested_start_time);
    }
    catch (...) {
      //ROS_INFO_STREAM("No requests for a different start time " << start_time_param_string);
    }

    try {
      ros::Time test_end_time = ros::Time::fromBoost(boost::posix_time::from_iso_extended_string(end_time_param_string));
      requested_end_time = test_end_time;
      ROS_INFO_STREAM("Requested end time " << end_time_param_string << " is " << requested_end_time);
    }
    catch (...) {
      //ROS_INFO_STREAM("No alternative end time is requested " << end_time_param_string);
    }

    // make sure the requested times fit inside the bag times
    if(requested_start_time == bag_start_time && requested_end_time == bag_end_time){
        if(!(start_percentage>=0 && start_percentage<100)){
            start_percentage = 0;
        }
        if(!(end_percentage<=100 && end_percentage>0)){
            end_percentage = 100;
        }
        auto duration_percentage = end_percentage - start_percentage;
        if(duration_percentage<100 && duration_percentage>0){
            ROS_INFO_STREAM("Reading bag from " << start_percentage << "% to " << end_percentage << "%");

            requested_start_time = bag_start_time + ros::Duration(bag_duration / 100 * start_percentage);
            requested_end_time = bag_start_time + ros::Duration(bag_duration / 100 * end_percentage);
        }
    }


    if(requested_start_time == bag_start_time){
      ROS_INFO_STREAM("Playback starts from the beginning: " << boost::posix_time::to_iso_extended_string(requested_start_time.toBoost()));
    } else {
      ROS_INFO_STREAM("Playback starts from: " << boost::posix_time::to_iso_extended_string(requested_start_time.toBoost()));
    }

    if(requested_end_time == bag_end_time){
      ROS_INFO_STREAM("Playback runs until the end: " << boost::posix_time::to_iso_extended_string(requested_end_time.toBoost()));
    } else {
      ROS_INFO_STREAM("Playback runs until: " << boost::posix_time::to_iso_extended_string(requested_end_time.toBoost()));
    }

    ROS_INFO_STREAM("Playback duration: " << requested_end_time-requested_start_time << " seconds");
    time_sync_real = ros::Time::now();
    time_sync_playback = requested_start_time;

    // tf static should be published by static tf broadcaster
    // so that if bag isn't played from begining, static will still be published
    for (auto bag: bags) {
      tf_static.StaticTfPublisher(bag->bag, true, transformer_);
    }

    // initialise the last packet time stored value
    last_packet_time = requested_start_time;

    // check if the user wants to publish the horizon transform
    private_nh.param<bool>("horizon_in_buffer", horizonInBuffer, false);

}



void h264_bag_playback::CloseBags() {
  bag_writer.CloseBags();
}




void h264_bag_playback::OpenBags() {

  // generate the views and advertise each of the topics to publish
  for (auto bag: bags) {
    bag->view = std::make_shared<rosbag::View>(bag->bag, rosbag::TopicQuery(std::vector<std::string>(bag->topics.begin(), bag->topics.end())), requested_start_time, requested_end_time);
    bag->iter = bag->view->begin();
    AdvertiseTopics(bag->view);
  }


  int dataset_event_count = 0;

  // find all of the dataset events
  for (auto bag: bags) {
    std::list<std::string> event_topics;

    for (auto topic: bag->topics) {
      if (topic.find("/event/") != std::string::npos) {
        //std::cout << "looking at event topics " << topic << " in bag " << bag->bag_file_name << std::endl;
        event_topics.push_back(topic);
      }
    }

    rosbag::View event_view(bag->bag, rosbag::TopicQuery(std::vector<std::string>(event_topics.begin(), event_topics.end())));

    for (auto message: event_view) {
      dataset_msgs::DatasetEvent::ConstPtr event_msg = message.instantiate<dataset_msgs::DatasetEvent>();
      if (event_msg) {
        if (dataset_events.find(message.getTopic()) == dataset_events.end()) {
          dataset_events[message.getTopic()] = std::list<dataset_msgs::DatasetEvent::ConstPtr>();
        }
        dataset_events[message.getTopic()].push_back(event_msg);
        dataset_event_count++;
        //std::cout << "including message " << event_msg->event_description << " from topic " << message.getTopic() << std::endl;
      }
    }
  }

  ROS_INFO_STREAM("Loaded " << dataset_event_count << " events");


  // creat a tf bag view object so that we can view future tf msgs
  std::vector<std::string> tf_topics{"tf", "/tf"};
  std::vector<std::string> imu_topics{"vn100/imu", "/vn100/imu"};


  for (auto bag: bags) {

    std::cout << "1 " << bag->bag_file_name << std::endl;

    for (auto tf_topic: tf_topics) {
      if (bag->topics.find(tf_topic) != bag->topics.end()) {
        ROS_INFO_STREAM("starting TF view from bag " << bag->bag_file_name);
        tf_view = std::make_shared<rosbag::View>(bag->bag, rosbag::TopicQuery(tf_topics), requested_start_time, requested_end_time);
      }
    }

    for (auto imu_topic: imu_topics) {
      if (bag->topics.find(imu_topic) != bag->topics.end()) {
        ROS_INFO_STREAM("starting IMU view from bag " << bag->bag_file_name);
        imu_view = std::make_shared<CorrectedImuPlayback>(bag->bag, rosbag::TopicQuery(imu_topics), requested_start_time, requested_end_time);
      }
    }
  }

  if (tf_view) {
    tf_iter = tf_view->begin();
  }

  last_tf_time = ros::Time(0.01);

  if (imu_view) {
    imu_view->ResetPlayback();
  }

  // calculate the total number of messages
  for (auto bag: bags) {
    if (bag->view)
      total_message_count += bag->view->size();
  }
}



void h264_bag_playback::SeekTime(ros::Time seek_time) {

  ros::Time earliest_time = ros::TIME_MAX;

  // if the seek time is less than the last_packet_time, reset the iterators to the beginning
  if (seek_time < last_packet_time) {
    for (auto bag: bags) {
      bag->iter = bag->view->begin();
      ROS_INFO_STREAM("restarting view on bag " << bag->bag_file_name);
    }
  }

  last_packet_time = seek_time;

  // find the next message in time order
  do {
    earliest_time = ros::TIME_MAX;

    std::shared_ptr <BagContainer> earliest_iter;
    bool valid_iter = false;

    for (auto bag: bags) {
      if (bag->iter == bag->view->end())
        continue;

      if (bag->iter->getTime() < earliest_time) {
        earliest_iter = bag;
        earliest_time = bag->iter->getTime();
        valid_iter = true;
      }
    }

    if (!valid_iter)
      return;

    earliest_iter->iter++;

  } while (earliest_time <= seek_time);
}




bool h264_bag_playback::ReadNextPacket() {

  // find the next in time order
  ros::Time earliest_time = ros::TIME_MAX;

  std::shared_ptr<BagContainer> earliest_iter;
  bool valid_iter = false;

  for (auto bag: bags) {
    if (bag->iter == bag->view->end())
      continue; //return false;
    if (bag->iter->getTime() < earliest_time) {
      earliest_iter = bag;
      earliest_time = bag->iter->getTime();
      valid_iter = true;
    }
  }

  if (!valid_iter)
    return false;

  rosbag::MessageInstance const m = *(earliest_iter->iter);
  earliest_iter->iter++;

  std::string const& topic = m.getTopic();

  if (topic == "/tf_static" || topic == "tf_static") {
    // static transforms are handled separately
    return true;
  }

  ros::Time const& time = m.getTime();

  last_packet_time = time;

  // all of the publishers should be available due to the AdvertiseTopics function
  std::map<std::string, ros::Publisher>::iterator pub_iter = publishers.find(m.getCallerId() + topic);
  ROS_ASSERT(pub_iter != publishers.end());

  // For each camera info msg, check whether we have stored the calibration parameters for this camera
  if (keep_last_of_string(topic, "/") == "camera_info") {
    std::string camera_name = keep_last_of_string(remove_last_of_string(topic, "/"), "/");

    sensor_msgs::CameraInfo::ConstPtr cam_info_msg = m.instantiate<sensor_msgs::CameraInfo>();
    if (cam_info_msg != NULL) {
      sensor_msgs::CameraInfo::Ptr scaled_info_msg(new sensor_msgs::CameraInfo());

      // copy the camera info parameters to the rescaled version
      *scaled_info_msg = *cam_info_msg;

      if (scaled_height && scaled_width) {
        Video::ScaleCameraInfoMsg(cam_info_msg->width,
                                  scaled_width,
                                  cam_info_msg->height,
                                  scaled_height,
                                  scaled_info_msg);
      }

      if (!videos[camera_name].valid_camera_info) {
        videos[camera_name].InitialiseCameraInfo(*scaled_info_msg);
      }

      // todo: make this go through the MessagePublisher structure
      //MessagePublisher(pub_iter->second, scaled_info_msg);
      //CameraInfoPublisher(pub_iter->second, m, scaled_info_msg);

      ros::spinOnce();
    }
  }

  // For each frame info msg, find the corresponding h.264 frame and publish/convert if necessary
  else if (keep_last_of_string(topic, "/") == "frame_info") {

    std::string camera_name = keep_last_of_string(remove_last_of_string(topic, "/"), "/");

    gmsl_frame_msg::FrameInfo::ConstPtr frame_info_msg = m.instantiate<gmsl_frame_msg::FrameInfo>();
    if (frame_info_msg != NULL) {

      ros::Time adjusted_image_stamp;

      if (frame_info_msg->camera_timestamp != 0) {
        ros::Time nvidia_timestamp = ros::Time((frame_info_msg->camera_timestamp) / 1000000.0, ((frame_info_msg->camera_timestamp) % 1000000) * 1000.0);

        // calculate the time bias between the camera/ROS if not already done
        if (!camera_time_bias_flag) {
          camera_time_bias = frame_info_msg->header.stamp - nvidia_timestamp;

          camera_time_bias_flag = true;
        }

  ////      std::cout << "calculating time bias for " << camera_name << " as " << camera_time_bias << ", " << frame_info_msg->header.stamp.toNSec() << ", " <<  nvidia_timestamp.toNSec() << std::endl;

        if (fabs(camera_time_bias.toSec()) > 0.5) {
          adjusted_image_stamp = nvidia_timestamp + camera_time_bias - time_offset_;
        }
        else {
          adjusted_image_stamp = nvidia_timestamp - time_offset_;
        }
      }
      else {
        adjusted_image_stamp = frame_info_msg->header.stamp;
      }


      // Check that someone has subscribed to this camera'frame_info_msg images
      if (play_all_cameras || (!(videos[camera_name].corrected_publisher.getNumSubscribers() == 0 &&
            videos[camera_name].uncorrected_publisher.getNumSubscribers() == 0)))
      {

        Video &current_video = videos[camera_name];

        // check that the frame counter aligns with the number of frames in the video
        if (current_video.SeekFrame(frame_info_msg->frame_counter)) {

          cv::Mat new_frame;

          if (scaled_height && scaled_width) {
            cv::Mat unresized_frame;
            current_video.video_device >> unresized_frame;

            cv::Size reduced_size = cv::Size(scaled_width, scaled_height);
            cv::resize(unresized_frame, new_frame, reduced_size);
          }
          else {
            current_video.video_device >> new_frame;
          }

          current_video.frame_counter++;

          //if (current_video.valid_camera_info) {


            // Check if someone wants the corrected (undistorted) camera images
            if (current_video.valid_camera_info && videos[camera_name].corrected_publisher.getNumSubscribers() > 0) {

              try {
                cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

                cv::Mat output_image;

                if (current_video.camera_info_msg.distortion_model == "rational_polynomial") {
                  cv::undistort(new_frame, output_image, current_video.camera_matrix, current_video.distance_coeffs);
                }
                else if (current_video.camera_info_msg.distortion_model == "equidistant") {
                  cv::remap(new_frame, output_image, current_video.map1, current_video.map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
                }
                else {
                  ROS_INFO_STREAM("Unknown distortion model, skipping " << current_video.camera_info_msg.distortion_model);
                  return true;
                }

                cv_ptr->image = output_image;
                cv_ptr->encoding = "bgr8";
                cv_ptr->header.stamp = adjusted_image_stamp;
                cv_ptr->header.frame_id = frame_id_dict[camera_name];
                cv_ptr->header.seq = frame_info_msg->global_counter;

                auto image_message = cv_ptr->toImageMsg();

                current_video.corrected_camera_info_msg.header = image_message->header;

                sensor_msgs::CameraInfo::ConstPtr new_info_message(new sensor_msgs::CameraInfo(current_video.corrected_camera_info_msg));

                CameraInfoPublisher(current_video.corrected_info_publisher, m, new_info_message);

                ImagePublisher(current_video.corrected_publisher, image_message);
                ros::spinOnce();
                //current_video.corrected_publisher.publish(cv_ptr->toImageMsg());
              }
              catch(...) {
                ROS_ERROR_STREAM("error in converting " << current_video.corrected_info_publisher.getTopic() << " to a rectified image");
              }
            }

            // Check if someone wants the uncorrected camera images
            if (play_all_cameras || videos[camera_name].uncorrected_publisher.getNumSubscribers() > 0) {
              cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

              cv_ptr->image = new_frame;
              cv_ptr->encoding = "bgr8";
              cv_ptr->header.stamp = adjusted_image_stamp;
              cv_ptr->header.frame_id = frame_id_dict[camera_name];
              cv_ptr->header.seq = frame_info_msg->global_counter;

              //current_video.uncorrected_publisher.publish(cv_ptr->toImageMsg());
              auto image_message = cv_ptr->toImageMsg();

              current_video.camera_info_msg.header = image_message->header;

              sensor_msgs::CameraInfo::ConstPtr new_info_message(new sensor_msgs::CameraInfo(current_video.camera_info_msg));
              CameraInfoPublisher(current_video.uncorrected_info_publisher, m, new_info_message);

              ImagePublisher(current_video.uncorrected_publisher, image_message);
              ros::spinOnce();
            }
          //}
        }
      }
    }

    // repubish the frame info message
    MessagePublisher(pub_iter->second, m);
    ros::spinOnce();
  }
  else if (topic == "vn100/imu" || topic == "/vn100/imu" || topic == "xsens/IMU" || topic == "/xsens/IMU") {

    auto msg = m.instantiate<sensor_msgs::Imu>();

    if (msg) {
      // compute horizon transforms from imu msg and publish them
      geometry_msgs::TransformStamped baselink, footprint;
      CorrectedImuPlayback::imu2horizontf(msg, baselink, footprint);

      tf_broadcaster.sendTransform(baselink);
      tf_broadcaster.sendTransform(footprint);

      auto header_time = msg->header.stamp;

      // query the bag tf msgs so that transformer buffers a tf tree from (current msg time - 8s) to (current msg time + 2s)
      // this however does not affect replay publishing of tf msgs. Tf msgs are still published at their bag times
      if (tf_view) {
        while (tf_iter != tf_view->end()
               && last_tf_time < header_time + ros::Duration(12.)) {

          // Load more transforms into the TF buffer
          auto tf_msg = tf_iter->instantiate<tf2_msgs::TFMessage>();

          if (tf_msg) {
            for (const auto &transform: tf_msg->transforms) {
              transformer_->setTransform(transform, "zio", false);
              last_tf_time = transform.header.stamp;
            }
            tf_iter++;
          }
        }
      }

      if (imu_view && horizonInBuffer) {
        imu_view->CalculateHorizon(transformer_, header_time);
      }
    }

    MessagePublisher(pub_iter->second, m);
    ros::spinOnce();
  }
  else {
    // publish the remaining messages
    //pub_iter->second.publish(m);
    MessagePublisher(pub_iter->second, m);
    ros::spinOnce();
  }


  if (limit_playback_speed && (topic == "tf" || topic == "/tf")) {

    if (ros::Time::now() - time_sync_latest > ros::Duration(.5)){
      // assume there has been a pause of the playback, the sync needs to be reset
      time_sync_real = ros::Time::now();
      time_sync_latest = ros::Time::now();
      time_sync_playback = time;

      ROS_INFO_STREAM("Real time re-sync now[" << ros::Time::now() <<"] last sync[" << time_sync_latest << "]");
    }
    else {

      double playback_time = (time - time_sync_playback).toSec();
      double real_time = (ros::Time::now() - time_sync_real).toSec();

      double scaled_time_difference = playback_time - real_time * scale_playback_speed;

      time_sync_latest = ros::Time::now();

      ros::Duration time_difference(scaled_time_difference);

      if (time_difference > ros::Duration(.5)) {
        // assume things are out of sync, reset the playback timer
        time_sync_real = ros::Time::now();
        time_sync_playback = time;

        time_difference = ros::Duration(0.);

        ROS_INFO_STREAM("Playback re-sync");
      }

      // hold back the playback to be realtime if limit_playback_speed parameter is set
      //if (limit_playback_speed && scaled_time_difference > 0) {
      if (scaled_time_difference > 0) {
        time_difference.sleep();
      }
    }
  }

  return true;
}


void h264_bag_playback::ReadFromBag() {

  OpenBags();

  while (ReadNextPacket())
  {
    // Spin once so that any other ros controls/pub/sub can be actioned
    ros::spinOnce();

    if (!ros::ok())
      break;
  }

  ROS_INFO_STREAM("completed playback");
}


void
h264_bag_playback::CameraInfoPublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message,
                                       const sensor_msgs::CameraInfoConstPtr &scaled_info_msg) {
  publisher.publish(scaled_info_msg);
}


void
h264_bag_playback::MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message) {
  publisher.publish(message);
}


void
h264_bag_playback::ImagePublisher(image_transport::Publisher &publisher, const sensor_msgs::ImageConstPtr &message) {
  publisher.publish(message);
}


void
h264_bag_playback::AdvertiseTopics(std::shared_ptr<rosbag::View> view) {

  // Create a publisher and advertise for all of our message types
  for(const rosbag::ConnectionInfo* c: view->getConnections())
  {
    // skip adding tf static. static should be published by static tf broadcaster
    // so that if bag isn't played from begining, static will still be published
    if (c->topic == "/tf_static" || c->topic == "tf_static") {
        continue;
    }

    ros::M_string::const_iterator header_iter = c->header->find("callerid");
    std::string callerid = (header_iter != c->header->end() ? header_iter->second : std::string(""));

    std::string callerid_topic = callerid + c->topic;

    std::map<std::string, ros::Publisher>::iterator pub_iter = publishers.find(callerid_topic);
    if (pub_iter == publishers.end()) {

      ros::AdvertiseOptions opts = rosbag::createAdvertiseOptions(c, 10);

      ros::Publisher pub = public_nh.advertise(opts);
      publishers.insert(publishers.begin(), std::pair<std::string, ros::Publisher>(callerid_topic, pub));
    }
  }
}


  PLUGINLIB_EXPORT_CLASS(dataset_toolkit::h264_bag_playback, nodelet::Nodelet);

}
