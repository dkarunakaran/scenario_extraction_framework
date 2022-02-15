#ifndef FEATURE_EXTRACTOR_HEADER
#define FEATURE_EXTRACTOR_HEADER

#include <map>
#include <chrono>

#include <ros/ros.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <visualization_msgs/Marker.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <h264_bag_playback/h264_bag_playback.hpp>
//#include <pluginlib/class_list_macros.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/PCLPointCloud2.h>

#include <custom_point_types/point_xyzir.h>
#include <custom_point_types/point_xyzirl.h>

#include <opencv/cv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "draw_shapes.hpp"
#include <lane_points_msg/LanePoints.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/algorithms/overlaps.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/multi/geometries/multi_linestring.hpp>
#include <boost/filesystem.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>

namespace bg = boost::geometry;
namespace filesys = boost::filesystem;
typedef bg::model::d2::point_xy<double> Point;
typedef bg::model::d2::point_xy<double> Vector;
typedef bg::model::segment<Point> Segment;
typedef bg::model::linestring<Point> LineString;
typedef bg::model::multi_linestring<LineString> MultiLineString;
typedef bg::model::multi_point<Point> MultiPoint;

class FeatureExtractor : public dataset_toolkit::h264_bag_playback
{

public:
  FeatureExtractor()
      : h264_bag_playback(),
        current_message_number(0),
        previous_percentage(-1)
  {
	  this->initialize();
  }

  void bypass_init() {
    this->onInit();
  }

  void MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message);
  void ImagePublisher(image_transport::Publisher &publisher, const sensor_msgs::ImageConstPtr &message) {}
  void CameraInfoPublisher(ros::Publisher &publisher, const sensor_msgs::CameraInfoConstPtr &message) {}
  void WriteImage();
  bool checkRegionOfInterest(std::pair<int,int>, int, int);
  
private:


  void initialize();

  void findLanePoints(std::vector<std::pair<long int, pcl::PointCloud<pcl::PointXYZI>>> vecPC);
  
  pcl::PointCloud<pcl::PointXYZI> processForSphericalCoordinateFrame(pcl::PointCloud<pcl::PointXYZIR>::Ptr input_cloud);

  std::vector<std::vector<float> > median (std::vector<std::vector<float> > matrixPC, int coord, int window);

  pcl::PointCloud<pcl::PointXYZI> extractEdges(pcl::PointCloud<pcl::PointXYZIR>::Ptr input_cloud, long int sec, long int nsec);

  int middlePoint(std::vector<std::vector<float>> matrixPC, float value);

  std::vector<std::vector<float> > findEdges(std::vector<std::vector<float> > matrixPC, float AngleThreshold, float Angle_d_Threshold, float IntensityThreshold, float Intensity, int middle_intensity_index, int points, std::vector<std::vector<float>>& obs_points);
 
  pcl::PointCloud<pcl::PointXYZI> mat2PCL(std::vector<std::vector<float> > matrixPC);

  double wrapAngle(double angle);
 
  pcl::PointCloud<pcl::PointXYZI> pointCloudFilter(pcl::PointCloud<pcl::PointXYZI>);

  void constructLaneSegments(pcl::PointCloud<pcl::PointXYZI> lanePC);

  bool checkVector(std::vector<std::pair<double, double>>, std::pair<double, double>);
  
  bool checkVector(std::vector<std::pair<std::vector<std::pair<double, double>>, int>>, std::pair<double, double>);

  void constructLane();
  
  std::pair<std::vector<double>,std::vector<double>> findODOMPoints();
  
  std::pair<std::vector<double>,std::vector<double>> findSlopeLaneSeg(std::vector<std::pair<double, double>>, double);
  
  void joinLaneSegment(std::tuple<Segment,double,double>, std::tuple<Segment,double,double>); 

  //std::tuple<double, double, double> linearRegression(std::vector<double> x, std::vector<double> y);
  
  void joinLinesFurther(std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>>);
  
  void removeNoiseLines(double); 

  void removeLineIntersects();

  void outliersRemoval(std::vector<std::pair<double, double>> laneSegment, std::vector<double>& xs, std::vector<double>& ys);

  std::tuple<Segment, double, double> getIntermediateSlope(Segment seg1, Segment seg2);

  void joinLinesFurther2(std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>>);

  void joinLinesFurther3(std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>>);
  
  void joinLinesFurther4(std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>>);
  
  void removeTheLineinConnectedPath();
  
  int IsPointInBoundingBox(float x1, float y1, float x2, float y2, float px, float py);
  
  std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>> joinEachSegInALine(std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>>);
 
  std::tuple<std::vector<double>, std::vector<double>, LineString> getOdomLocation(Point point1, Point point2);

  void removeIntersectingLines();

  void cleanLineStrings();
  
  lanelet::LineString3d linestringToLineString3d(LineString ls);
  
  void removeDuplicates();
  
  bool checkDuplicates(LineString , double, std::vector<std::pair<double, LineString>>);
  
  bool isLeft(lanelet::Point3d, lanelet::Point3d, lanelet::Point3d);
  
  void createLaneletMap();
  
  LineString segPairsToLineString(std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>>);
  
  // initialise ros stuff
  virtual void onInit();

  std::vector<std::string> separateCommas(std::string input_string);

  // call whenever receive a pointcloud - spit out new filtered version
  void SegmentPointCloud_intensity(sensor_msgs::PointCloud2::ConstPtr pointcloud_msg,
                                   std::map<std::pair<int,int>, double> &intensity_topic);

  void SegmentPointCloud_label(sensor_msgs::PointCloud2::ConstPtr pointcloud_msg,
                               std::map<std::pair<int,int>, double> &intensity_topic);

  pcl::PointCloud<pcl::PointXYZIR>::Ptr Selector(pcl::PointCloud<pcl::PointXYZIR>::Ptr input_cloud,
                                                 float maximum_range,
                                                 std::set<int> ring_filter,
                                                 std::set<int> &rings_included);
  pcl::PointCloud<pcl::PointXYZIRL>::Ptr Selector_label(pcl::PointCloud<pcl::PointXYZIRL>::Ptr input_cloud,
                                                        float maximum_range,
                                                        std::set<int> ring_filter,
                                                        std::set<int> &rings_included);


  std::map<std::string, std::map<std::pair<int,int>, double>> intensity_map;
  std::list<std::pair<int,int>> vehicle_odom;

  std::vector<std::shared_ptr<DrawItem>> item_draw_properties;

  std::set<int> rings_included;
  std::set<int> ring_filter;

  std::string projection_frame;

  int cm_resolution;

  // variables to estimate the time remaining to perform the projection
  uint32_t current_message_number;
  int previous_percentage;
  std::chrono::steady_clock::time_point start_time, end_time;
  //std::vector<std::pair<int, int>> allPoints;
  ros::NodeHandle n;
  ros::Publisher lanePcPub;
  ros::Publisher signalShutdown;
  std::map<std::pair<int,int>, std::map<std::pair<int, int>, double>> allPoints;
  std::vector<long int> secWatch;
  int seq_count;
  std::map<std::pair<int, int>, std::pair<long int, pcl::PointCloud<pcl::PointXYZIR>::Ptr>> pointCloudPerNsec;
  std::map<long int, pcl::PointCloud<pcl::PointXYZIR>::Ptr> pointCloudBL;
  std::list<std::pair<double,double>> lane_odom;
  std::list<std::pair<int,int>> lane_points;
  std::vector<std::pair<double,double>> vehicle_odom_double;
  std::vector<std::tuple<std::vector<std::pair<double, double>>, int, std::vector<std::pair<double, double>>>> active_lane_segments;//[lane points in lane segments, inactive count]
  std::vector<std::vector<std::pair<double, double>>> lane_segments;//[lane points lane segements]
  std::vector<std::pair<std::vector<double>,std::vector<double>>> lanes;
  std::vector<std::pair<std::vector<double>,std::vector<double>>> checkLanes;
  std::vector<std::vector<std::pair<double,double>>> boundingBoxes;
  std::vector<std::pair<std::vector<std::pair<double, double>>, int>> activeLanes; //[lane segments, inactive count]
  std::vector<std::tuple<Segment,double,double>> lines;
  std::vector<std::pair<std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>>,int>> activeLaneSeg;
  std::vector<std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>>> allLines;
  std::vector<std::pair<std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>>,int>> activeLaneSeg2;
  std::vector<std::pair<std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>>,int>> activeLaneSeg3;
  std::vector<std::pair<std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>>,int>> activeLaneSeg4;
  std::vector<LineString> allLineStrings;
  std::string laneletsmap_file;
  std::string laneletToOD_file;
  std::vector<std::pair<LineString, int>> activeLaneSeg5;
  std::vector<std::pair<lanelet::LineString3d, int>> activeLineString3d;
  std::vector<lanelet::LineString3d> allLineStrings3d;
  std::vector<std::pair<lanelet::LineString3d, int>> activeLineString3d1;
  lanelet::LineString3d odomLineString;
  std::vector<lanelet::LineString3d> activeLS3d;
  ros::Publisher finishPub;
};


#endif
