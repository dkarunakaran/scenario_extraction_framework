  #include <gtest/gtest.h>

  #include <iostream>
  #include "boost/date_time/posix_time/posix_time.hpp"

  #include <math.h>

  #include "h264_bag_playback.hpp"
  #include "bag_data.h"


  class DirectPlayback : public dataset_toolkit::h264_bag_playback {

  public:

    DirectPlayback(std::string bag_file)
        : h264_bag_playback() {

      private_nh.setParam("bag_file", bag_file);
      init_playback();
    }


    void ImagePublisher(image_transport::Publisher &publisher, const sensor_msgs::ImageConstPtr &message) {}


    void CameraInfoPublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message,
                                       const sensor_msgs::CameraInfoConstPtr &scaled_info_msg){}


    void MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message) {
      if (stats.find(message.getTopic()) == stats.end()) {
        stats[message.getTopic()] = 0;
      }

      stats[message.getTopic()]++;
    }
    std::map<std::string, int> stats;
  };



  class BagReaderTest : public testing::Test {
  protected:

    virtual void SetUp() {
    }

  };


  TEST_F(BagReaderTest, test_full_bag_read) {

    std::string bag_file_name = std::string(TEST_DATA_LOCATION) + std::string("/pipeline.bag");
    DirectPlayback playback(bag_file_name);

    // clear params
    playback.private_nh.setParam("time_start", "");
    playback.private_nh.setParam("time_end", "");
    playback.private_nh.setParam("percentage_start", 0.);
    playback.private_nh.setParam("percentage_end", 100.);

    playback.init_playback();


    playback.ReadFromBag();

    for(auto elem : playback.stats){
      std::cout << elem.first << " " << elem.second << std::endl;
    }

    // checked from test file using rosbag info
    EXPECT_EQ(playback.stats["/vn100/imu"], 266);
    EXPECT_EQ(playback.stats["/tf"], 114);
    EXPECT_EQ(playback.stats["/velodyne_points"], 26);
  }

  TEST_F(BagReaderTest, test_full_bag_read_with_params) {

    std::string bag_file_name = std::string(TEST_DATA_LOCATION) + std::string("/pipeline.bag");
    DirectPlayback playback(bag_file_name);
    playback.private_nh.setParam("time_start", "2018-02-18T22:42:40.30");
    playback.private_nh.setParam("time_end", "2018-02-18T22:42:43.0");
    playback.init_playback();
    playback.ReadFromBag();

    // checked from test file using rosbag info
    EXPECT_EQ(playback.stats["/vn100/imu"], 266);
    EXPECT_EQ(playback.stats["/tf"], 114);
    EXPECT_EQ(playback.stats["/velodyne_points"], 26);

  }

  TEST_F(BagReaderTest, test_late_start) {

  std::string bag_file_name = std::string(TEST_DATA_LOCATION) + std::string("/pipeline.bag");
  DirectPlayback playback(bag_file_name);
  playback.private_nh.setParam("time_start", "2018-02-18T22:42:41.30");
  playback.private_nh.setParam("time_end", "");
  playback.init_playback();
  playback.ReadFromBag();

  // checked from test file using rqt_bag
  EXPECT_LT(playback.stats["/vn100/imu"], 200);
  EXPECT_LT(playback.stats["/tf"], 100);
  EXPECT_EQ(playback.stats["/velodyne_points"], 17);

  }

  TEST_F(BagReaderTest, test_early_end) {

  std::string bag_file_name = std::string(TEST_DATA_LOCATION) + std::string("/pipeline.bag");
  DirectPlayback playback(bag_file_name);
  playback.private_nh.setParam("time_start", "");
  playback.private_nh.setParam("time_end", "2018-02-18T22:42:41.30");
  playback.init_playback();
  playback.ReadFromBag();

  // checked from test file using rqt_bag
  EXPECT_LT(playback.stats["/vn100/imu"], 200);
  EXPECT_LT(playback.stats["/tf"], 100);
  EXPECT_EQ(playback.stats["/velodyne_points"], 9);

  }

  TEST_F(BagReaderTest, test_percentage_start) {

  std::string bag_file_name = std::string(TEST_DATA_LOCATION) + std::string("/pipeline.bag");
  DirectPlayback playback(bag_file_name);
  playback.private_nh.setParam("time_start", "");
  playback.private_nh.setParam("time_end", "");
  playback.private_nh.setParam("percentage_start", 99.8);
  playback.private_nh.setParam("percentage_end", -0.1);
  playback.init_playback();
  playback.ReadFromBag();

  for(auto elem : playback.stats){
  std::cout << elem.first << " " << elem.second << std::endl;
  }

  // checked from test file using rqt_bag
  EXPECT_LT(playback.stats["/vn100/imu"], 50);
  EXPECT_LT(playback.stats["/tf"], 50);
  EXPECT_LT(playback.stats["/velodyne_points"], 9);

  }

  TEST_F(BagReaderTest, test_percentage_end) {

  std::string bag_file_name = std::string(TEST_DATA_LOCATION) + std::string("/pipeline.bag");
  DirectPlayback playback(bag_file_name);
  playback.private_nh.setParam("time_start", "");
  playback.private_nh.setParam("time_end", "");
  playback.private_nh.setParam("percentage_start", NAN);
  playback.private_nh.setParam("percentage_end", 0.1);
  playback.init_playback();
  playback.ReadFromBag();

  for(auto elem : playback.stats){
  std::cout << elem.first << " " << elem.second << std::endl;
  }

  // checked from test file using rqt_bag
  EXPECT_LT(playback.stats["/vn100/imu"], 50);
  EXPECT_LT(playback.stats["/tf"], 50);
  EXPECT_LT(playback.stats["/velodyne_points"], 9);

  }


  TEST_F(BagReaderTest, test_percentage_mid) {

  std::string bag_file_name = std::string(TEST_DATA_LOCATION) + std::string("/pipeline.bag");
  DirectPlayback playback(bag_file_name);
  playback.private_nh.setParam("time_start", "");
  playback.private_nh.setParam("time_end", "");
  playback.private_nh.setParam("percentage_start", 0.1);
  playback.private_nh.setParam("percentage_end", 0.2);
  playback.init_playback();
  playback.ReadFromBag();

  for(auto elem : playback.stats){
  std::cout << elem.first << " " << elem.second << std::endl;
  }

  // checked from test file using rqt_bag
  EXPECT_LT(playback.stats["/vn100/imu"], 50);
  EXPECT_LT(playback.stats["/tf"], 50);
  EXPECT_LT(playback.stats["/velodyne_points"], 9);

  }

  int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;

    return RUN_ALL_TESTS();
  }
