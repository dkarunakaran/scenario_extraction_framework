#ifndef VIDEO_HEADER
#define VIDEO_HEADER

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <gmsl_frame_msg/FrameInfo.h>


class Video {
public:

  Video() {}

  bool InitialiseVideo(std::string camera_name, std::string video_filename);
  void InitialiseCameraInfo(sensor_msgs::CameraInfo &camera_info);

  bool valid_camera_info;

  // scale the camera info message for a different output size
  static void ScaleCameraInfoMsg(int original_width,
                          int scaled_width,
                          int original_height,
                          int scaled_height,
                          sensor_msgs::CameraInfo::Ptr &scaled_info_msg);


  bool SeekFrame(uint32_t requested_frame);

  cv::Mat camera_matrix, distance_coeffs, map1, map2;

  std::string file_name;

  sensor_msgs::CameraInfo camera_info_msg, corrected_camera_info_msg;

  cv::VideoCapture video_device;
  uint32_t frame_counter;


  ros::Publisher corrected_info_publisher;
  ros::Publisher uncorrected_info_publisher;

  image_transport::Publisher corrected_publisher;
  image_transport::Publisher uncorrected_publisher;
};





#endif