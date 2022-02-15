#include "laneletsmap_generator.hpp"

#include <pcl/search/impl/search.hpp>
#include <pcl/filters/crop_box.h>

#include <iostream>
#include <vector>
#include <sstream>

#include <deque>

#include <Eigen/Dense>

#include <boost/shared_ptr.hpp>

#include <pcl/common/common.h>
#include <pcl/common/angles.h>
#include <pcl/common/transforms.h>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <signal.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include "json.hpp"
#include "Conversions.h"
#include <pcl/common/io.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types_conversion.h>
#include <random>
#include <Eigen/QR>
#include <unistd.h>
using json = nlohmann::json;
#include "helper_functions.hpp"

int main(int argc, char **argv) {

  ros::init(argc, argv, "TopDownProjection");
  ros::NodeHandle n;
  FeatureExtractor pole_detect;
  pole_detect.bypass_init();
  ros::spin();  
  
  return 0;
}

void FeatureExtractor::initialize(){
	seq_count = 0;
}



std::vector <std::string>
FeatureExtractor::separateCommas(std::string input_string) {
  std::vector <std::string> result;
  std::stringstream s_stream(input_string); //create string stream from the string
  while (s_stream.good()) {
    std::string substr;
    std::getline(s_stream, substr, ','); //get first string delimited by comma
    result.push_back(substr);
  }

  return result;
}


void
FeatureExtractor::onInit() {


  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  private_nh.param<int>("cm_resolution", cm_resolution, 2);


  private_nh.param<std::string>("projection_frame", projection_frame, "odom");

  std::vector<float> filter_ring_numbers;
  private_nh.getParam("use_rings", filter_ring_numbers);

  for (auto &ring: filter_ring_numbers) {
    ROS_INFO_STREAM("Incorporating lidar ring in image:" << ring);
    ring_filter.insert(ring);
  }

  std::vector <std::string> topics;

  std::vector <std::string> value_dict;
  private_nh.getParam("point_clouds", value_dict);

  for (auto &iter: value_dict) {
    std::vector <std::string> values = separateCommas(iter);

    if (values.size() != 4)
      ROS_INFO_STREAM("incomplete type to display topic " << values[0]);
    else {
      if (values[1] == "circle") {

        std::shared_ptr <CircleItem> new_item = std::make_shared<CircleItem>(std::stoi(values[2]),
                                                                             std::stoi(values[3]));
        new_item->topic_name = values[0];
        new_item->alpha = std::stoi(values[2]);
        new_item->shape_size = std::stoi(values[3]);
        item_draw_properties.push_back(new_item);
        ROS_INFO_STREAM("Drawing topic " << item_draw_properties.back()->topic_name
                                         << " as a circle with alpha/size " << item_draw_properties.back()->alpha
                                         << ", "
                                         << item_draw_properties.back()->shape_size << ", using field "
                                         << new_item->field_name);
      } else if (values[1] == "point") {

        std::shared_ptr <PointItem> new_item = std::make_shared<PointItem>(std::stoi(values[2]),
                                                                           std::stoi(values[3]));
        new_item->topic_name = values[0];
        new_item->alpha = std::stoi(values[2]);
        new_item->shape_size = std::stoi(values[3]);
        item_draw_properties.push_back(new_item);
        ROS_INFO_STREAM("Drawing topic " << item_draw_properties.back()->topic_name
                                         << " as a point with alpha/size " << item_draw_properties.back()->alpha
                                         << ", "
                                         << item_draw_properties.back()->shape_size << ", using field "
                                         << new_item->field_name);
      } else if (values[1] == "square") {

      } else {
        ROS_INFO_STREAM("Drawing topic " << values[0] << " has no type defined");
        continue;
      }
      topics.push_back(values[0]);

    }
  }

  for (auto &topic: topics) {
    ROS_INFO_STREAM(topic << " is being used");
    intensity_map[topic] = std::map < std::pair < int, int >, double > ();
  }

  private_nh.getParam("laneletsmap_file", laneletsmap_file);
  private_nh.getParam("laneletToOD_file", laneletToOD_file);
  finishPub = n.advertise<std_msgs::String>("finish_map_generation", 1);  
  this->horizonInBuffer = true;
  this->init_playback();
  start_time = std::chrono::steady_clock::now();
  this->ReadFromBag();
  this->constructLane();
  this->WriteImage();
  
  sleep(5);

  //Publishing the msg to inform the end of this process
  std_msgs::String msg;
  msg.data = "true";
  finishPub.publish(msg);

}

void FeatureExtractor::constructLane(){
  ROS_INFO_STREAM("constructLane function called");
  ROS_INFO_STREAM("All line size before activeLaneSeg: "<<allLineStrings.size());  
  /*for(size_t i=0; i<activeLaneSeg.size(); i++){
    auto lineSegVec = activeLaneSeg[i].first;
    joinLinesFurther(lineSegVec);
  }
  for(size_t i=0; i<activeLaneSeg2.size(); i++){
    auto lineSegVec = activeLaneSeg2[i].first;
    joinLinesFurther2(lineSegVec);
  }
  for(size_t i=0; i<activeLaneSeg3.size(); i++){
    auto lineSegVec = activeLaneSeg3[i].first;
    joinLinesFurther3(lineSegVec);
  }*/

  for(size_t i=0; i<activeLaneSeg.size(); i++){
    auto lineSegVec = activeLaneSeg[i].first;
    auto ls = segPairsToLineString(lineSegVec);
    allLineStrings.push_back(ls);
  }

  /*for(size_t i=0; i<activeLaneSeg2.size(); i++){
    auto lineSegVec = activeLaneSeg2[i].first;
    auto ls = segPairsToLineString(lineSegVec);
    allLineStrings.push_back(ls);
  }
  for(size_t i=0; i<activeLaneSeg3.size(); i++){
    auto lineSegVec = activeLaneSeg3[i].first;
    auto ls = segPairsToLineString(lineSegVec);
    allLineStrings.push_back(ls);
  }
  for(size_t i=0; i<activeLaneSeg4.size(); i++){
    auto lineSegVec = activeLaneSeg4[i].first;
    auto ls = segPairsToLineString(lineSegVec);
    allLineStrings.push_back(ls);
  }*/

  ROS_INFO_STREAM("All line size after activeLaneSeg3: "<<allLineStrings.size());  
  /*Post processing the lines*/
  //Some lines has multiple line segmets with same end points which gives
  //multiple lines between first and last point of the lines.
  cleanLineStrings();
  ROS_INFO_STREAM("No. of lines: "<<allLineStrings.size());
  //Removing duplicate lines
  removeDuplicates();
  ROS_INFO_STREAM("No. of lines: "<<allLineStrings.size());
  removeIntersectingLines();
  ROS_INFO_STREAM("No. of lines: "<<allLineStrings.size());
  ROS_INFO_STREAM("No. of lines: "<<allLineStrings.size());
  RemoveCloserLinesOnLeftOrRight(allLineStrings);
  removeNoiseLines(10.);
  //The below logic to join the lines can be only applied to straightroad.
  //If it's not a straight road, we have to come up with different logic
  //joincleanLineSegements(allLineStrings, 1.0, 50., 5.);
  //cleanLineStrings();
  
  //joincleanLineSegements(allLineStrings, 1.0, 50., 5.);
  //cleanLineStrings();
  //checkIntersect(allLineStrings); 
  
  ROS_INFO_STREAM("No. of lines after all processing: "<<allLineStrings.size());
  
  createLaneletsMap(allLineStrings, vehicle_odom_double, laneletsmap_file, laneletToOD_file, bag_file_name);

}


bool FeatureExtractor::isLeft(lanelet::Point3d a, lanelet::Point3d b, lanelet::Point3d c){
  //d=(x−x1)(y2−y1)−(y−y1)(x2−x1)   
  auto d = ((c.x() - a.x())*(b.y() - a.y())) - ((c.y() - a.y())*(b.x() - a.x()));
  
  return d > 0;
}

void FeatureExtractor::removeDuplicates(){
  auto tempLineStrings = allLineStrings;
  allLineStrings.clear();
  std::vector<std::pair<double, LineString>> lines;
  for(size_t i=0; i < tempLineStrings.size(); i++){
    auto line = tempLineStrings[i];
    auto length = bg::length(line);
    if(line.size() == 0){
      lines.push_back(std::make_pair(length, line));
      allLineStrings.push_back(line);
    }else{
      if(!checkDuplicates(line, length, lines)){
        lines.push_back(std::make_pair(length, line));
        allLineStrings.push_back(line);
      }
    }
  }
  tempLineStrings.clear();
}

bool FeatureExtractor::checkDuplicates(LineString checkLine, double checkLength, std::vector<std::pair<double, LineString>> lines){
  bool result = false;
  auto checkP1 = checkLine[0];
  auto checkP2 = checkLine[checkLine.size()-1];
  for(size_t i=0; i<lines.size(); i++){
    auto length = lines[i].first;
    auto line = lines[i].second;
    if(length == checkLength){
      auto p1 = line[0];
      auto p2 = line[line.size()-1];
      if(p1.get<0>() == checkP1.get<0>() && p1.get<1>() == checkP1.get<1>() && p2.get<0>() == checkP2.get<0>() && p2.get<1>() == checkP2.get<1>())
        result = true;
    }
  }
  
  return result;
}

lanelet::LineString3d FeatureExtractor::linestringToLineString3d(LineString ls){
  lanelet::LineString3d ls3d(lanelet::utils::getId(), {});
  for(auto& point: ls){
    ls3d.push_back(lanelet::Point3d(lanelet::utils::getId(), point.get<0>(), point.get<1>(), 0));
  }

  return ls3d;
}

void FeatureExtractor::cleanLineStrings(){
  auto temp = allLineStrings;  
  allLineStrings.clear();
  for(size_t i=0; i<temp.size(); i++){
    auto line = temp[i];
    std::vector<std::vector<double>> sortedVec;
    std::vector<double> xs;
    for(auto& point: line){
      std::vector<double> row;
      row.push_back(point.get<0>());
      row.push_back(point.get<1>());
      sortedVec.push_back(row);
    }
    std::sort(sortedVec.begin(), sortedVec.end(), [](const std::vector<double>& a, const std::vector<double>& b) {return a[0] > b[0];});
    LineString linestring;
    for(auto& vec: sortedVec){
      linestring.push_back(Point(vec[0], vec[1])); 
    }
    allLineStrings.push_back(linestring);
  }
  allLines.clear();

}

void FeatureExtractor::removeIntersectingLines(){
  auto lineStrings = allLineStrings;
  allLineStrings.clear();
  std::vector<size_t> eraseVec;
  for(size_t i=0; i<lineStrings.size(); i++){
    auto linestring1 = lineStrings[i];
    bool found = false;
    std::vector<size_t> listInter;
    for(size_t j=0; j<lineStrings.size(); j++){
      if(std::find(eraseVec.begin(), eraseVec.end(), j) != eraseVec.end()|| i == j)
       continue; 
      auto linestring2 = lineStrings[j];
      MultiLineString intersection;
      bg::intersection(linestring1, linestring2, intersection);
      if(intersection.size() > 0){
        listInter.push_back(j);
      }
    }
    
    //Remove intersecting lines
    auto d1 = bg::length(linestring1);
    if(d1 < 10 && listInter.size() == 1){
      auto linestring2 = lineStrings[listInter[0]];
      auto d2 = bg::length(linestring2);
      if(d2 > 10)
        eraseVec.push_back(i);
    }else{
    }
  
    //Removing small lines 
    if(d1 < 1.){
      if(std::find(eraseVec.begin(), eraseVec.end(), i) != eraseVec.end())
        continue;
      else
        eraseVec.push_back(i);
    }
  }
  
  for(size_t i=0; i<lineStrings.size(); i++){
    if(std::find(eraseVec.begin(), eraseVec.end(), i) != eraseVec.end()){
      continue;
    }else{
      allLineStrings.push_back(lineStrings[i]);
    }
  }
}


void FeatureExtractor::WriteImage() {

  ROS_INFO_STREAM("Number of plotted topics: " << intensity_map.size());

  // set the search values to extreme (to be overwritten by the correct values)
  int max_x = -100000000, max_y = -100000000, min_x = 100000000, min_y = 100000000;
  float min_intensity = 100000000.;
  float max_intensity = -100000000.;

  for (auto &topic_intensity: intensity_map) {
    // determine the min/max values for x/y pixels and intensity
    for (auto &entry: topic_intensity.second) {
      min_x = std::min<int>(min_x, entry.first.first);
      max_x = std::max<int>(max_x, entry.first.first);

      min_y = std::min<int>(min_y, entry.first.second);
      max_y = std::max<int>(max_y, entry.first.second);

      min_intensity = std::min<double>(min_intensity, entry.second);
      max_intensity = std::max<double>(max_intensity, entry.second);
    }
  }

  std::cout << min_x << ", " << min_y << ", " << max_x << ", " << max_y << std::endl;

  // restrict the intensity range as determined by the param settings
  float intensity_scale_min = private_nh.param<float>("min_intensity", 0.);
  float intensity_scale_max = private_nh.param<float>("max_intensity", 80.);

  min_intensity = std::max<float>(min_intensity, intensity_scale_min);
  max_intensity = std::min<float>(max_intensity, intensity_scale_max);

  // Add a small buffer for the image to account for rounding errors
  int image_max_x = max_x - min_x + 2;
  int image_max_y = max_y - min_y + 2;

  // the lidar datapoints projected into 2d
  cv::Mat output_image(image_max_x, image_max_y, CV_8UC4, cv::Scalar(0, 0, 0, 0));
  
  /*
  for (auto &topic_to_draw: item_draw_properties) {
    auto topic_intensity = intensity_map.find(topic_to_draw->topic_name);
    if (topic_intensity == intensity_map.end()) {
      ROS_INFO_STREAM(topic_to_draw->topic_name << " is not found to draw");
      break;
    }

    ROS_INFO_STREAM(topic_intensity->first << " has " << topic_intensity->second.size() << " values");
    for (auto &entry: topic_intensity->second) {

      int value_x = entry.first.first - min_x;
      int value_y = entry.first.second - min_y;

      if (value_x < 0 || value_x >= image_max_x) {
        ROS_ERROR_STREAM("Pixel found out of bounds: " << intensity_map.size());
        continue;
      }

      if (value_y < 0 || value_y >= image_max_y) {
        ROS_ERROR_STREAM("Pixel found out of bounds: " << intensity_map.size());
        continue;
      }

      cv::Vec4b colour;

      // if sematic label is being plotted, use bonnet color scheme
      if (topic_to_draw->field_name == "label") {

        int label = entry.second;
        colour[3] = (uint8_t)(255);

        if (label == 1) {//building
          colour[0] = (uint8_t)(255);
          colour[1] = (uint8_t)(255);
          colour[2] = (uint8_t)(255);
        } else if (label == 2) {//pole
          colour[2] = (uint8_t)(158);
          colour[1] = (uint8_t)(255);
          colour[0] = (uint8_t)(255);
        } else if (label == 3) {//road
          colour[2] = (uint8_t)(139);
          colour[1] = (uint8_t)(69);
          colour[0] = (uint8_t)(19);
        } else if (label == 4) {//undrivable_road
          colour[2] = (uint8_t)(202);
          colour[1] = (uint8_t)(255);
          colour[0] = (uint8_t)(111);
        } else if (label == 5) {//vegetation
          colour[2] = (uint8_t)(0);
          colour[1] = (uint8_t)(255);
          colour[0] = (uint8_t)(0);
        } else if (label == 6) {//sign
          colour[2] = (uint8_t)(158);
          colour[1] = (uint8_t)(255);
          colour[0] = (uint8_t)(255);
        } else if (label == 7) {//fence
          colour[2] = (uint8_t)(160);
          colour[1] = (uint8_t)(160);
          colour[0] = (uint8_t)(160);
        } else if (label == 8) {//vehicle
          colour[2] = (uint8_t)(255);
          colour[1] = (uint8_t)(0);
          colour[0] = (uint8_t)(0);
        } else {
          colour[3] = (uint8_t)(0);
        }

        // else use rainbow color scheme for plotting intensity
      } else {

        // scale the intensity value to be between 0 and 1
        double intensity = (entry.second - min_intensity) / (max_intensity - min_intensity);

        if (intensity > 1.)
          intensity = 1.;

        if (intensity < 0.)
          intensity = 0.;

        // set the hue to the intensity value (between 0 and 255) to make a rainbow colour scale
        hsv input_hsv;
        input_hsv.h = intensity * 255.;
        input_hsv.s = 1.;
        input_hsv.v = 1.;

        // convert HSV to RGB
        rgb output_rgb = hsv2rgb(input_hsv);

        //colour[0] = (uint8_t)(output_rgb.b * 255.);
        //colour[1] = (uint8_t)(output_rgb.g * 255.);
        //colour[2] = (uint8_t)(output_rgb.r * 255.);
	      colour[0] = 0.;
	      colour[1] = 255.;
	      colour[2] = 0.;

        //std::cout << "point " << value_x << ", " << value_y << " intensity " << intensity << std::endl;
      }

      cv::Point destination_point(value_y, value_x);
      topic_to_draw->drawItem(output_image, destination_point, colour);
    }
  }
  */
  // Set the colour for the odom plot
  cv::Scalar odom_colour(255., 50., 0., 180);
  float odom_radius = 5.;
  /*
  // Draw a circle for each of the odom positions
  for (auto &odom: vehicle_odom) {
    int value_x = odom.first - min_x;
    int value_y = odom.second - min_y;
    cv::circle(output_image, cv::Point(value_y, value_x), odom_radius, odom_colour, CV_FILLED);
  }*/

  cv::Scalar odom_colour1(0., 0., 255., 180);
  float odom_radius1 = 1.;
  cv::Scalar lane_point_colour(255., 255., 255., 180);
  for(auto& points: lane_points){
    int value_x = points.first - min_x;
    int value_y = points.second - min_y;
    cv::circle(output_image, cv::Point(value_y, value_x), 1, lane_point_colour, CV_FILLED);

  }
  
  int thickness = 1;
  for(auto& linestring: allLineStrings){
    cv::Scalar color1(
      (double)std::rand() / RAND_MAX * 255,
      (double)std::rand() / RAND_MAX * 255,
      (double)std::rand() / RAND_MAX * 255,
      255
    );
    cv::Scalar color2(0., 0., 255., 180);
    bool first = true;
    for(size_t i=0; i<linestring.size()-1; i++){
      auto point1 = linestring[i]; auto point2 = linestring[i+1];
      double x1 = point1.get<0>(); double y1 = point1.get<1>();
      double x2 = point2.get<0>(); double y2 = point2.get<1>();
      int x_index1 = ((y1*100)/cm_resolution)*-1;int y_index1 = (x1*100)/cm_resolution;
      int value_x1 = x_index1 - min_x;int value_y1 = y_index1 - min_y;
      int x_index2 = ((y2*100)/cm_resolution)*-1;int y_index2 = (x2*100)/cm_resolution;
      int value_x2 = x_index2 - min_x;int value_y2 = y_index2 - min_y;
      cv::Point p1(value_y1, value_x1);cv::Point p2(value_y2, value_x2);
      cv::line(output_image, p1, p2, color1, thickness, cv::LINE_8);
      if(first){
        cv::circle(output_image, cv::Point(value_y1, value_x1), 2., color2, CV_FILLED);
        first = false;
      }
    }
  }
  /*
  int thickness = 1;
  for(auto& linestring: allLineStrings3d){
    cv::Scalar color1(
      (double)std::rand() / RAND_MAX * 255,
      (double)std::rand() / RAND_MAX * 255,
      (double)std::rand() / RAND_MAX * 255,
      255
    );
    cv::Scalar color2(0., 0., 255., 180);
    bool first = true;
    for(size_t i=0; i<linestring.size()-1; i++){
      auto point1 = linestring[i]; auto point2 = linestring[i+1];
      double x1 = point1.x(); double y1 = point1.y();
      double x2 = point2.x(); double y2 = point2.y();
      int x_index1 = ((y1*100)/cm_resolution)*-1;int y_index1 = (x1*100)/cm_resolution;
      int value_x1 = x_index1 - min_x;int value_y1 = y_index1 - min_y;
      int x_index2 = ((y2*100)/cm_resolution)*-1;int y_index2 = (x2*100)/cm_resolution;
      int value_x2 = x_index2 - min_x;int value_y2 = y_index2 - min_y;
      cv::Point p1(value_y1, value_x1);cv::Point p2(value_y2, value_x2);
      cv::line(output_image, p1, p2, color1, thickness, cv::LINE_8);
      if(first){
        cv::circle(output_image, cv::Point(value_y1, value_x1), 2., color2, CV_FILLED);
        first = false;
      }
    }
  }*/


  for (auto &ring_number: rings_included) {
    ROS_INFO_STREAM("Ring " << ring_number << " included in image");
  }

  ROS_INFO_STREAM(
      "Drawing image with bounds: [" << min_x / 100. * cm_resolution << ", " << min_y / 100. * cm_resolution
                                     << "],  [" <<
                                     max_x / 100. * cm_resolution << ", " << max_y / 100. * cm_resolution
                                     << "] with intensity range [" <<
                                     min_intensity << ", " << max_intensity << "]");

  // output the white background image
  std::string output_image_name = private_nh.param<std::string>("output_image", "");


  if (output_image_name != "")
    cv::imwrite(output_image_name, output_image);



  double x_datum = 0, y_datum = 0;

  try {

    // transform from the world reference to the base link
    auto datum_transform = transformer_->lookupTransform(std::string("utm"),
                                                         projection_frame,
                                                         ros::Time(0));

    Eigen::Vector3f datum_origin(datum_transform.transform.translation.x,
                                 datum_transform.transform.translation.y,
                                 datum_transform.transform.translation.z);

    ROS_INFO_STREAM("DATUM TRANSFORM (to utm): " << datum_origin[0] << ", " << datum_origin[1]);

    //double x_datum = 332722.272927207, y_datum = 6248431.02677212;
    x_datum = datum_origin[0];
    y_datum = datum_origin[1];

    std::cout << "gdal_translate -of GTiff -co \"COMPRESS=JPEG\" -a_srs EPSG:32756 -a_ullr " <<
              std::setprecision(15) << x_datum + cm_resolution * double(min_y) / 100. << " " <<
              std::setprecision(15) << y_datum + -cm_resolution * double(min_x) / 100. << " " <<
              std::setprecision(15) << x_datum + cm_resolution * double(max_y) / 100. << " " <<
              std::setprecision(15) << y_datum + -cm_resolution * double(max_x) / 100. << " " <<
              output_image_name <<
              " output-georeferenced.tif" << std::endl;

  } catch (const std::exception &e) { // reference to the base of a polymorphic object
    ROS_ERROR_STREAM(e.what()); // information from length_error printed
    ROS_INFO_STREAM("No datum (transform to utm) is available");
  }

}

pcl::PointCloud<pcl::PointXYZI> FeatureExtractor::extractEdges(pcl::PointCloud<pcl::PointXYZIR>::Ptr input_cloud, long int sec, long int nsec){

  pcl::PointCloud<pcl::PointXYZI> pointCloud;
  std::vector<std::vector<float>>  ring1;
	std::vector<std::vector<float>>  ring2;
	std::vector<std::vector<float>>  ring3;

	// Sort the pointclouds
	for (size_t i = 0; i < input_cloud->points.size(); ++i) {
		std::vector<float> row;
    row.push_back(input_cloud->points[i].x); 
		row.push_back(input_cloud->points[i].y); 
		row.push_back(input_cloud->points[i].z); 
		row.push_back(input_cloud->points[i].intensity);
		if(input_cloud->points[i].ring == 90){
			ring1.push_back(row);
		}else if(input_cloud->points[i].ring == 91){
			ring2.push_back(row);
		}else if(input_cloud->points[i].ring == 92){
			ring3.push_back(row);
		}
	}

  if(ring1.size() != 0 && ring2.size() != 0 && ring3.size() != 0){

    //Sorting	
    std::sort(ring1.begin(), ring1.end(), [](const std::vector<float>& a, const std::vector<float>& b) {return a[1] < b[1];});
    std::sort(ring2.begin(), ring2.end(), [](const std::vector<float>& a, const std::vector<float>& b) {return a[1] < b[1];});
    std::sort(ring3.begin(), ring3.end(), [](const std::vector<float>& a, const std::vector<float>& b) {return a[1] < b[1];});
    int middleR1 = middlePoint(ring1, 0);
    int middleR2 = middlePoint(ring2, 0);
    int middleR3 = middlePoint(ring3, 0);
    int middleR4 = 0;
    
    std::vector<std::vector<float> > obs_points;
    // 15 10
    std::vector<std::vector<float> > edge1 = findEdges(ring1, 15, 10, 9, ring1[middleR1][3], middleR1, 15, obs_points);// 12 10
    std::vector<std::vector<float> > edge2 = findEdges(ring2, 15, 10, 9, ring2[middleR2][3], middleR2, 14, obs_points);// 12 10
    std::vector<std::vector<float> > edge3 = findEdges(ring3, 15, 10, 9, ring3[middleR3][3], middleR3, 10, obs_points);// 12 10

    pcl::PointCloud<pcl::PointXYZI> edge_pc1 = mat2PCL(edge1);
    pcl::PointCloud<pcl::PointXYZI> edge_pc2 = mat2PCL(edge2);
    pcl::PointCloud<pcl::PointXYZI> edge_pc3 = mat2PCL(edge3);
    pcl::PointCloud<pcl::PointXYZI> obs = mat2PCL(obs_points);
    pcl::PointCloud<pcl::PointXYZI> total;
    
    total += edge_pc1;
    total += edge_pc2;
    total += edge_pc3;

    pcl::PCLPointCloud2 cloudR1;pcl::PCLPointCloud2 cloudR2;
    pcl::PCLPointCloud2 cloudR3;//pcl::PCLPointCloud2 cloudR4;
    pcl::PCLPointCloud2 cloudFinal;pcl::PCLPointCloud2 obs_pc;
    pcl::toPCLPointCloud2(edge_pc1,cloudR1);pcl::toPCLPointCloud2(edge_pc2,cloudR2);
    pcl::toPCLPointCloud2(edge_pc3,cloudR3);//pcl::toPCLPointCloud2(edge_pc4,cloudR4);
    pcl::toPCLPointCloud2(obs,obs_pc);

    float height2 = 0.20;float height3 = 0.25;//float height4 = 0.47;
        if (ring2[middleR2][2]<height2){
            pcl::concatenatePointCloud (cloudR2, cloudR1, cloudFinal);
            if (ring3[middleR3][2]<height3){
              pcl::concatenatePointCloud (cloudFinal, cloudR3, cloudFinal);
            }
        }
    
    //sensor_msgs::PointCloud2 roadPC;

    if(total.points.size() > 500){
      pcl::PointCloud<pcl::PointXYZI> tempCloud;
      pcl::fromPCLPointCloud2(cloudFinal, pointCloud);
    }else{
      auto cloudFiltered = processForSphericalCoordinateFrame(input_cloud);
      pcl::copyPointCloud(cloudFiltered, pointCloud);
    }
  }// main if closes	
	
  return pointCloud;
}

pcl::PointCloud<pcl::PointXYZI> FeatureExtractor::mat2PCL(std::vector<std::vector<float> > matrixPC){
     pcl::PointCloud<pcl::PointXYZI> pointCloud;
     for (int i=0;i < matrixPC.size(); i=i+1){
       pcl::PointXYZI point;
       point.x = matrixPC[i][0];
       point.y = matrixPC[i][1];
       point.z = matrixPC[i][2];
       point.intensity = matrixPC[i][3];
       pointCloud.push_back(point);
     }
     return pointCloud;
}


std::vector<std::vector<float>> FeatureExtractor::findEdges(std::vector<std::vector<float> > matrixPC, float AngleThreshold, float Angle_d_Threshold, float IntensityThreshold, float Intensity, int middle_intensity_index, int points, std::vector<std::vector<float>>& obs_points) {
     std::vector<std::vector<float> > edges_points;
     int n=20;
     int inclination_change=0;
     float angle;
     float anglexy;
     
     bool obs_N_det= true;
     
     //Angle on the xy palne
     auto angle_past_xy = atan2((matrixPC[middle_intensity_index+points][0]-matrixPC[middle_intensity_index][0]),((matrixPC[middle_intensity_index+points][1]-matrixPC[middle_intensity_index][1])))*180/3.14159265;
     
     //Angle on the yz plane
     auto angle_past = atan2((matrixPC[middle_intensity_index+points][2]-matrixPC[middle_intensity_index][2]),((matrixPC[middle_intensity_index+points][1]-matrixPC[middle_intensity_index][1])))*180/3.14159265;
     
     //One side of the points
     for (int i=middle_intensity_index; i< (matrixPC.size()-points); i=i+1)
     {
        std::vector<float> edge_h;

        //Angle on the xy palne
              anglexy=atan2((matrixPC[i+points][0]-matrixPC[i][0]),((matrixPC[i+points][1]-matrixPC[i][1])))*180/3.14159265;
              
        //Angle on the yz palne
        angle=atan2((matrixPC[i+points][2]-matrixPC[i][2]),((matrixPC[i+points][1]-matrixPC[i][1])))*180/3.14159265;
            
        //ROS_INFO_STREAM("Angle D " << angle << ".\n");
            
          //comparing angle
          if ((std::abs(angle))<AngleThreshold && (std::abs(angle-angle_past))<Angle_d_Threshold && obs_N_det ){
            edge_h.push_back(matrixPC[i][0]);
            edge_h.push_back(matrixPC[i][1]);
            edge_h.push_back(matrixPC[i][2]);
            edge_h.push_back(matrixPC[i][3]);
            edges_points.push_back(edge_h);
            angle_past = angle;
            angle_past_xy =anglexy;
          } else {
            obs_N_det= false;
            //std::cout << "Angle I " << angle << ".\n";
            //std::cout << "Angle Diff " << (angle-angle_past) << ".\n";
            edge_h.push_back(matrixPC[i][0]);
            edge_h.push_back(matrixPC[i][1]);
            edge_h.push_back(matrixPC[i][2]);
            edge_h.push_back(matrixPC[i][3]);
            obs_points.push_back(edge_h);
          }
      }

     // OTHER SIDE OF THE POINTS
     obs_N_det= true;
     
      //Angle on the yz palne
     angle_past = atan2((matrixPC[middle_intensity_index][2]-matrixPC[middle_intensity_index-points][2]),((matrixPC[middle_intensity_index][1]-matrixPC[middle_intensity_index-points][1])))*180/3.14159265;
     for (int i=middle_intensity_index; i> points; i=i-1)
     {
        std::vector<float> edge_h;

	      //Angle on the yz palne
        angle=atan2((matrixPC[i][2]-matrixPC[i-points][2]),((matrixPC[i][1]-matrixPC[i-points][1])))*180/3.14159265;
        
	      //comparing angle
        //cout << "Angle I " << angle << ".\n";
        if ((std::abs(angle))<AngleThreshold && (std::abs(angle-angle_past))<Angle_d_Threshold && obs_N_det ){
            edge_h.push_back(matrixPC[i][0]);edge_h.push_back(matrixPC[i][1]);edge_h.push_back(matrixPC[i][2]);edge_h.push_back(matrixPC[i][3]);
            edges_points.push_back(edge_h);
            angle_past=angle;
         } else {
             obs_N_det= false;
           //  cout << "Izquierda .\n";
           //  cout << "Angle I " << angle << ".\n";
           //  cout << "Angle Diff " << (angle-angle_past) << ".\n";
          edge_h.push_back(matrixPC[i][0]);edge_h.push_back(matrixPC[i][1]);edge_h.push_back(matrixPC[i][2]);edge_h.push_back(matrixPC[i][3]);
          obs_points.push_back(edge_h);
          // break;
       }
     }
     
     return edges_points;
}


int FeatureExtractor::middlePoint(std::vector<std::vector<float>> matrixPC, float value){
    int middle;
    float middle_f = 100000;
    for (int i=0; i< (matrixPC.size()-1); i=i+1){
      if ((std::abs(matrixPC[i][1]-value))<middle_f){
        middle = i;
        middle_f = std::abs(matrixPC[i][1]-value);
      }
    }
    
    return (middle);
}

pcl::PointCloud<pcl::PointXYZI> FeatureExtractor::processForSphericalCoordinateFrame(pcl::PointCloud<pcl::PointXYZIR>::Ptr input_cloud){
	//ROS_INFO_STREAM("processForSphericalCoordinateFrame");
	/*Converting cartisian to spherical coordinate frame.
	 *Spherical is 3d for of  2d polar cartician frame. Polar is used in c		ircle. Plar has radis and angle and working on xy plane as it is 2d.
	 *More detailed explnation can be found here: https://blog.demofox.org/2013/10/12/converting-to-and-from-polar-spherical-coordinates-made-easy/
	 *Equation of spherical coordinate frames
	 *radius = sqrt(X * X + Y * Y + Z * Z) //distance
	 *theta = atan2(Y, X) // bearing
	 *phi = acos(Z / radius) //pitch
	*/

        pcl::PointCloud<pcl::PointXYZI> pointCloud;
	pcl::copyPointCloud(*input_cloud, pointCloud);
	
	float radiusThreshold_max = 7.0;
	float angleThreshold_min = 2.0;
	float angleThreshold_max = 4.25;
	std::vector<std::vector<float>>  matrix;
	std_msgs::Float32MultiArray r_array, t_array, p_array;
	pcl::PointCloud<pcl::PointXYZI> temp_cloud; 
	for (size_t i = 0; i < pointCloud.points.size(); ++i) {
      		auto x = pointCloud.points[i].x;
      		auto y = pointCloud.points[i].y;
      		auto z = pointCloud.points[i].z;
		auto radius = sqrt(pow(x, 2)+pow(y, 2)+pow(z, 2));
		auto theta = atan2(y, x);
		auto phi = acos((z/radius));
		theta = wrapAngle(theta);
		
		if(theta < angleThreshold_min || theta > angleThreshold_max)
			continue;
		
		if(radius > radiusThreshold_max)
			continue;
          
		pcl::PointXYZI point;
		point.x = x;
		point.y = y;
		point.z = z;
		point.intensity = pointCloud.points[i].intensity;
		temp_cloud.push_back(point);
		
		r_array.data.push_back(radius);
		t_array.data.push_back(theta);
		p_array.data.push_back(phi);
	}

	return temp_cloud;

}

double FeatureExtractor::wrapAngle( double angle )
{
    double twoPi = 2.0 * 3.141592865358979;
    return angle - twoPi * floor( angle / twoPi );
}

bool FeatureExtractor::checkRegionOfInterest(std::pair<int,int> item, int min_x, int min_y){

  bool _return = false;

  // Draw a circle for each of the odom positions
  for (auto &odom: vehicle_odom) {
    int x1 = odom.first - min_x;
    int y1 = odom.second - min_y;
    
    
    int x2 = item.first;
    int y2 = item.second;
    float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
    d = (cm_resolution*d)/100;
     
    if(d<=3){
      _return = true;
      break;
    }
  }

  return _return;
}


void FeatureExtractor::MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message) {

  current_message_number++;

  int current_percentage = 100.0 * ((float) current_message_number / (float) total_message_count);

  if (current_percentage != previous_percentage) {
    previous_percentage = current_percentage;

    if (current_percentage == 0) {
      std::cout << "starting projection" << std::endl;
    } else {
      end_time = std::chrono::steady_clock::now();
      std::chrono::duration<float, std::ratio<60>> elapsed = end_time - start_time;
      float estimated_remaining = (float) (100 - current_percentage) * elapsed.count();
      std::cout << current_percentage << "%, est. remaining " << estimated_remaining << " minutes" << std::endl;
    }
    start_time = std::chrono::steady_clock::now();
  }

  sensor_msgs::PointCloud2::ConstPtr s = message.instantiate<sensor_msgs::PointCloud2>();
  if (s != NULL) {
    auto map_reference = intensity_map.find(message.getTopic());
    if (map_reference != intensity_map.end()) {

      if (s->fields.size() > 5 && std::string(s->fields[5].name) == std::string("label")) {
        SegmentPointCloud_label(s, map_reference->second);
      } else
        SegmentPointCloud_intensity(s, map_reference->second);
    }
  }
}


pcl::PointCloud<pcl::PointXYZIR>::Ptr
FeatureExtractor::Selector(pcl::PointCloud<pcl::PointXYZIR>::Ptr input_cloud,
                           float maximum_range,
                           std::set<int> ring_filter,
                           std::set<int> &rings_included) {

  float max_range_squared = pow(maximum_range, 2);

  pcl::PointCloud<pcl::PointXYZIR>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZIR>);

  for (size_t i = 0; i < input_cloud->points.size(); ++i) {
    float range_squared =
        pow(input_cloud->points[i].x, 2) + pow(input_cloud->points[i].y, 2) + pow(input_cloud->points[i].z, 2);

   
    if (range_squared > max_range_squared)
      continue;


    int ring_number = (int) (input_cloud->points[i].ring);

    if (ring_filter.size() > 0) {
      if (ring_filter.find(ring_number) == ring_filter.end()) {
        ROS_INFO_STREAM_THROTTLE(1., "ignoring data from ring " << ring_number);
        continue;
      }
    }

    rings_included.insert(ring_number);

    downsampled->points.push_back(input_cloud->points[i]);
  }

  return downsampled;
}


pcl::PointCloud<pcl::PointXYZIRL>::Ptr
FeatureExtractor::Selector_label(pcl::PointCloud<pcl::PointXYZIRL>::Ptr input_cloud,
                                 float maximum_range,
                                 std::set<int> ring_filter,
                                 std::set<int> &rings_included) {

  float max_range_squared = pow(maximum_range, 2);

  pcl::PointCloud<pcl::PointXYZIRL>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZIRL>);

  for (size_t i = 0; i < input_cloud->points.size(); ++i) {
    float range_squared =
        pow(input_cloud->points[i].x, 2) + pow(input_cloud->points[i].y, 2) + pow(input_cloud->points[i].z, 2);

    if (range_squared > max_range_squared)
      continue;


    int ring_number = (int) (input_cloud->points[i].ring);

    if (ring_filter.size() > 0) {
      if (ring_filter.find(ring_number) == ring_filter.end()) {
        continue;
      }
    }

    rings_included.insert(ring_number);

    downsampled->points.push_back(input_cloud->points[i]);
  }

  return downsampled;
}

pcl::PointCloud<pcl::PointXYZI>  FeatureExtractor::pointCloudFilter(pcl::PointCloud<pcl::PointXYZI> inputCloud){
	

	//-------------------------Filter based on height-----------------------------
	float min_intensity = 100000000.;
	float max_intensity = -100000000.;
  std::vector<std::vector<float>> matrixPC;
  for (size_t i = 0; i < inputCloud.points.size(); ++i) {
          std::vector<float> row;
          row.push_back(inputCloud.points[i].x);
          row.push_back(inputCloud.points[i].y);
          row.push_back(inputCloud.points[i].z);
          row.push_back(inputCloud.points[i].intensity);
          matrixPC.push_back(row);
    min_intensity = std::min<double>(min_intensity, inputCloud.points[i].intensity);
    max_intensity = std::max<double>(max_intensity, inputCloud.points[i].intensity);

	}

	// Sorting
  std::sort(matrixPC.begin(), matrixPC.end(), [](const std::vector<float>& a, const std::vector<float>& b) {return a[1] < b[1];});


	std::vector<std::vector<float>> heightMatrixPC;
	int middle = middlePoint(matrixPC, 0);
	double heightDeltaThreshold = 0.10; //0.10
	double heightThreshold = 0.10; //0.20
	double previousHeight = 0.;
	
	//One side of the points
     	for (int i=middle; i<matrixPC.size(); i++)
     	{
		double z = matrixPC[i][2];
		double heightDelta = z-previousHeight;
		if(z < heightThreshold && heightDelta < heightDeltaThreshold){
			std::vector<float> row;
			row.push_back(matrixPC[i][0]);
			row.push_back(matrixPC[i][1]);
			row.push_back(matrixPC[i][2]);
			row.push_back(matrixPC[i][3]);
			heightMatrixPC.push_back(row);
			previousHeight = z;
		}
		
	}

	//Other side of the points
  for (int i=middle; i>0; i--)
  {
      double z = matrixPC[i][2];
      double heightDelta = z-previousHeight;
      if(z < heightThreshold && heightDelta < heightDeltaThreshold){
        std::vector<float> row;
        row.push_back(matrixPC[i][0]);
        row.push_back(matrixPC[i][1]);
        row.push_back(matrixPC[i][2]);
        row.push_back(matrixPC[i][3]);
        heightMatrixPC.push_back(row);
        previousHeight = z;
      }

  }

	//ROS_INFO_STREAM("Size of heightPC: "<<heightMatrixPC.size());

	//--------------------Filter based on intensity-------------------	

	// High intensity  = 70% higher
	auto intensity_min = (max_intensity*70/100); //70
	
	// Sorting
  std::sort(heightMatrixPC.begin(), heightMatrixPC.end(), [](const std::vector<float>& a, const std::vector<float>& b) {return a[1] < b[1];});

	
	pcl::PointCloud<pcl::PointXYZI> intensityPC;
	double intThreshold = 0.40; //0.40	
	bool deltaFlag = false;
	double previousInt = 0.;
	
	for(int i=0; i<heightMatrixPC.size();i++){
		double x = heightMatrixPC[i][0];
    double y = heightMatrixPC[i][1];
    double z = heightMatrixPC[i][2];
    double intensity = heightMatrixPC[i][3];
                        
    // Finding changes in intensity  
    double intDelta = intensity-previousInt;        
		if(intDelta > intThreshold && intensity >= intensity_min){
			deltaFlag = true;

		}else{
			deltaFlag = false;
		}	
		
		if(deltaFlag){
			pcl::PointXYZI point;
      point.x = x;
      point.y = y;
      point.z = z;
      point.intensity = intensity;
      intensityPC.push_back(point);
		}
		previousInt = intensity;
	}
	
  // -----------Averaging the pointcloud-------------------
	pcl::PointCloud<pcl::PointXYZI> lanePC;
	if(intensityPC.points.size() > 1) {	

		//pcl::copyPointCloud(intensityPC, lanePC);
    
    std::vector<std::vector<float>> avgMatrixPC;
		for (size_t i = 0; i < intensityPC.points.size(); ++i) {
			std::vector<float> row;
			row.push_back(intensityPC.points[i].x);
			row.push_back(intensityPC.points[i].y);
			row.push_back(intensityPC.points[i].z);
			row.push_back(intensityPC.points[i].intensity);
			avgMatrixPC.push_back(row);
		}
		
		// Sorting
		std::sort(avgMatrixPC.begin(), avgMatrixPC.end(), [](const std::vector<float>& a, const std::vector<float>& b) {return a[1] < b[1];});
    pcl::PointCloud<pcl::PointXYZI>::Ptr intensityPCPtr(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::copyPointCloud(intensityPC, *intensityPCPtr);
		pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
		kdtree.setInputCloud (intensityPCPtr); 
		double radius = 2.; //2 works
    std::vector<std::pair<double, double>> searchVec;
		for (int i=0; i<avgMatrixPC.size(); i++)
		{
      auto x1 = avgMatrixPC[i][0];
      auto y1 = avgMatrixPC[i][1];
      if(checkVector(searchVec, std::make_pair(x1,y1)))
        continue; 
      std::vector<std::vector<float>> knnMatrixPC;
      std::vector<float> row;
      row.push_back(avgMatrixPC[i][0]);
      row.push_back(avgMatrixPC[i][1]);
      row.push_back(avgMatrixPC[i][2]);
      row.push_back(avgMatrixPC[i][3]); 
			knnMatrixPC.push_back(row);
      for(int j=0; j<avgMatrixPC.size(); j++){
        auto x2 = avgMatrixPC[j][0]; 
        auto y2 = avgMatrixPC[j][1];
        if(i != j){
          float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
          if(d <= radius){
						row.clear();
						row.push_back(avgMatrixPC[i][0]);
            row.push_back(avgMatrixPC[i][1]);
            row.push_back(avgMatrixPC[i][2]);
            row.push_back(avgMatrixPC[i][3]); 
            knnMatrixPC.push_back(row);
            searchVec.push_back(std::make_pair(x2, y2));
          }
        }
      }
      //searchVec.push_back(std::make_pair(x1, y1));
      std::sort(knnMatrixPC.begin(), knnMatrixPC.end(), [](const std::vector<float>& a, const std::vector<float>& b) {return a[1] < b[1];});
      
      int middlePoint = knnMatrixPC.size()/2;
      pcl::PointXYZI point;
      point.x = knnMatrixPC[middlePoint][0];
      point.y = knnMatrixPC[middlePoint][1];
      point.z = knnMatrixPC[middlePoint][2];
      point.intensity = knnMatrixPC[middlePoint][0];
      lanePC.push_back(point);
		}
	}else{
    pcl::copyPointCloud(intensityPC, lanePC);
	}

	return lanePC;

}

bool FeatureExtractor::checkVector(std::vector<std::pair<double, double>> searchVec, std::pair<double, double> xy){
  bool status = false;  
  for(size_t i=0; i<searchVec.size(); i++){
    auto point = searchVec[i];
    auto x2 = point.first;
    auto y2 = point.second;
    if(xy.first == x2 && xy.second == y2){
      status = true;
      break;
    }
  }
  
  return status;
}


bool FeatureExtractor::checkVector(std::vector<std::pair<std::vector<std::pair<double, double>>, int>> searchVec, std::pair<double, double> xy){
  bool status = false;  
  for(size_t i=0; i<searchVec.size(); i++){
    auto segementActiveOrInactive = active_lane_segments[i];
    auto laneSegment = std::get<0>(segementActiveOrInactive);
    for(size_t j=0; j<searchVec.size(); j++){
      auto point = laneSegment[j];
      auto x2 = point.first;
      auto y2 = point.second;
      if(xy.first == x2 && xy.second == y2){
        status = true;
        break;
      }
    }
  }
  
  return status;
}

std::pair<std::vector<double>,std::vector<double>> FeatureExtractor::findODOMPoints(){
  std::pair<std::vector<double>,std::vector<double>> _return;
  std::vector<double> xs;std::vector<double> ys;
  if(vehicle_odom_double.size() > 1){
    auto x1 = vehicle_odom_double[vehicle_odom_double.size()-1].first;
    auto y1 = vehicle_odom_double[vehicle_odom_double.size()-1].second;
    xs.push_back(x1);
    ys.push_back(y1);
    for(size_t i=vehicle_odom_double.size()-2;i>=0;i--){
        auto x2 = vehicle_odom_double[i].first;
        auto y2 = vehicle_odom_double[i].second;
        xs.push_back(x2);
        ys.push_back(y2);
        float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
        if(d >= 1.){
          break;
        }
    }
    _return = std::make_pair(xs, ys);
  }
  
  return _return;
}

std::pair<std::vector<double>,std::vector<double>> FeatureExtractor::findSlopeLaneSeg(std::vector<std::pair<double, double>> laneSeg, double threshold){
  std::pair<std::vector<double>,std::vector<double>> lanePoints;
  auto x1 = laneSeg[laneSeg.size()-1].first;
  auto y1 = laneSeg[laneSeg.size()-1].second;
  for(size_t i=laneSeg.size()-2;i>=0;i--){
      auto x2 = laneSeg[i].first;
      auto y2 = laneSeg[i].second;
      float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
      if(d >= threshold){
        std::vector<double> row1;
        row1.push_back(x1);
        row1.push_back(y1);
        std::vector<double> row2;
        row2.push_back(x2);
        row2.push_back(y2);
        lanePoints = std::make_pair(row1, row2); 
        break;
      }
  }
  
  return lanePoints;
}

void FeatureExtractor::outliersRemoval(std::vector<std::pair<double, double>> laneSegment, std::vector<double>& xs, std::vector<double>& ys){
	pcl::PointCloud<pcl::PointXYZ>::Ptr laneSegPC(new pcl::PointCloud<pcl::PointXYZ>);
  for(auto& lSeg: laneSegment){
    pcl::PointXYZ point;
    point.x = lSeg.first;
    point.y = lSeg.second;
    point.z = 0.;
    laneSegPC->push_back(point);
  }
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (laneSegPC);
  sor.setMeanK (30);
  sor.setStddevMulThresh (0.5);
  sor.filter (*cloud_filtered);
  for(size_t i = 0; i < cloud_filtered->points.size(); ++i) {
			xs.push_back(cloud_filtered->points[i].x);
			ys.push_back(cloud_filtered->points[i].y);
	}
}


void FeatureExtractor::constructLaneSegments(pcl::PointCloud<pcl::PointXYZI> lanePC){
  //std::vector<std::pair<std::vector<std::pair<double, double>>, int>> active_lane_segments;//[lane po    ints in lane segments, inactive count]
  //std::map<long int, std::vector<std::pair<double, double>>> lane_segment;//[lane points lane segements]
  //ROS_INFO_STREAM("_____________________________________: "<<seq_count);
  //ROS_INFO_STREAM("Active lane segments size: "<<active_lane_segments.size());
  //ROS_INFO_STREAM("Lane segments size: "<<lane_segments.size());
  //ROS_INFO_STREAM("lanPC size: "<<lanePC.points.size());

  /*for(size_t i=0;i<lanePC.points.size();i++){
    auto x1 = lanePC.points[i].x;
    auto y1 = lanePC.points[i].y;
    std::vector<std::pair<double, double>> laneSegment;
    laneSegment.push_back(std::make_pair(x1, y1));  
    lane_segments.push_back(laneSegment);
  }*/
  float RADIUS = 1.; //1. works
  float INACTIVECOUNT = 20; //20 and below only works - tried with 50, and 100, but not getting good result
  if(active_lane_segments.size() == 0){
    std::vector<std::pair<double, double>> searchVec; 
    for(size_t i=0;i<lanePC.points.size();i++){
      auto x1 = lanePC.points[i].x;
      auto y1 = lanePC.points[i].y;
      //ROS_INFO_STREAM("active is zero: "<<x1<<" "<<y1);
      if(checkVector(searchVec, std::make_pair(x1,y1))) 
        continue;
      for(size_t j=0;j<lanePC.points.size();j++){
        auto x2 = lanePC.points[j].x;
        auto y2 = lanePC.points[j].y;
        if(i==j || checkVector(searchVec, std::make_pair(x2,y2)))
          continue;
        float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
        auto y_dist = std::abs(std::abs(y2)-std::abs(y1));
        
        //Avoiding closest lateral points
        if(d<RADIUS){
          searchVec.push_back(std::make_pair(x2, y2));
        }
      }

      if(!checkVector(searchVec, std::make_pair(x1,y1))) {
          std::vector<std::pair<double, double>> laneSegment;
          laneSegment.push_back(std::make_pair(x1, y1));
          std::vector<std::pair<double, double>> odomPoints;
          odomPoints.push_back(std::make_pair(vehicle_odom_double[vehicle_odom_double.size()-1].first, vehicle_odom_double[vehicle_odom_double.size()-1].second));
          active_lane_segments.push_back(std::make_tuple(laneSegment, 0, odomPoints));
      }

    }
   
  }else{
    std::vector<std::pair<double, double>> searchVec;
    std::vector<std::pair<double, double>> newPointsVec;
    std::vector<size_t> activeList;
    for(size_t i=0;i<lanePC.points.size();i++){
      auto x1 = lanePC.points[i].x;
      auto y1 = lanePC.points[i].y;
      double smallRadius = 10000.;
      size_t activeIndex = 100000;
      bool found = false;
      for(size_t j=0;j<active_lane_segments.size(); j++){
        if (std::find(activeList.begin(), activeList.end(), j) != activeList.end()) 
          continue;
        
        auto laneSegment = std::get<0>(active_lane_segments[j]);
        auto point = laneSegment.back();
        auto x2 = point.first; auto y2 = point.second;
        float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
        if(d<RADIUS && d<smallRadius){
          smallRadius = d;
          activeIndex = j;
          found = true;
        } 
      }//first active lane segment for loop close
      
      if(found){
        auto laneSegment = std::get<0>(active_lane_segments[activeIndex]);
        auto odomPoints = std::get<2>(active_lane_segments[activeIndex]);
        laneSegment.push_back(std::make_pair(x1, y1));
        odomPoints.push_back(std::make_pair(vehicle_odom_double[vehicle_odom_double.size()-1].first, vehicle_odom_double[vehicle_odom_double.size()-1].second));
        active_lane_segments[activeIndex] = std::make_tuple(laneSegment, 0, odomPoints);
        activeList.push_back(activeIndex);
      }else{
        newPointsVec.push_back(std::make_pair(x1, y1));
      }

    }//lanePC for loop close
    for(size_t j=0;j<active_lane_segments.size(); j++){
        if(std::find(activeList.begin(), activeList.end(), j) != activeList.end()) 
          continue;
        else{
          auto laneSegment = std::get<0>(active_lane_segments[j]);
          auto inactiveCount = std::get<1>(active_lane_segments[j]);
          auto odomPoints = std::get<2>(active_lane_segments[j]);
          inactiveCount += 1;
        
          // Remove from active and store them in lane segement
          if(inactiveCount > INACTIVECOUNT){
            bool proceed = true;
            std::vector<double> xs;
            std::vector<double> ys;
            if(laneSegment.size()< 5){
              for(auto& lSeg: laneSegment){
                xs.push_back(lSeg.first);
                ys.push_back(lSeg.second);
              }
            }else
              outliersRemoval(laneSegment, xs, ys);
            auto lineDetails = linearRegression(xs, ys);
            std::vector<double> odomX; std::vector<double> odomY;
            for(auto& odom: odomPoints){
              odomX.push_back(odom.first);
              odomY.push_back(odom.second);
            } 
            auto odomLineDetails = linearRegression(odomX, odomY);
            double m; double c;
            if(xs.size() > 2){ // 2 works
              auto lineDetails = linearRegression(xs, ys);
              auto mse = std::get<2>(lineDetails);
              m = std::get<0>(lineDetails);
              c = std::get<1>(lineDetails);
              //ROS_INFO_STREAM("m: "<<m<<" c: "<<c<<" mse: "<<mse);
              if(mse>0.04){ //0.05 works fine
                proceed = false;
              }

              //Also checking the slope of odom and lane odom
              auto x1 = odomPoints[0].first;
              auto y1 = odomPoints[0].second;
              auto x2 = odomPoints[odomPoints.size()-1].first;
              auto y2 = odomPoints[odomPoints.size()-1].second;
              float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
              if(odomPoints.size() > 2 && d > 0.15){
                auto slopeOdom =  std::get<0>(odomLineDetails);
                auto slopeDiff = std::abs(slopeOdom-m);
                //ROS_INFO_STREAM("slopeOdom: "<<slopeOdom<<" lineOdom: "<<m<<" diff: "<<slopeDiff);
                if(slopeDiff > 1.) //1.good for 5 to 8
                  proceed = false;
              }

            }else{
                //discard lanSegments that has only one point.
                proceed = false;
            }
            
            // Push the value to lane_segments
            if(proceed){
                lane_segments.push_back(laneSegment);
                //We have equation of the line in the form y=mx+b. It would be
                //great to calculate the start and end points by taking x value
                //from first and last point in the lane segment. 
                auto x1 = laneSegment[0].first;
                auto y1 = m*x1+c;
                auto x2 = laneSegment[laneSegment.size()-1].first;
                auto y2 = m*x2+c;
                Segment seg(Point(x1, y1), Point(x2, y2));
                //lines.push_back(std::make_tuple(seg, m,c));
                auto ox1 = odomPoints[0].first;
                auto oy1 = odomPoints[0].second;
                auto ox2 = odomPoints[odomPoints.size()-1].first;
                auto oy2 = odomPoints[odomPoints.size()-1].second;
                Segment odomSeg(Point(ox1, oy1), Point(ox2, oy2)); 
                auto odomLocPoints = getOdomLocation(Point(x1, y1), Point(x2, y2));
                auto odomDetails = linearRegression(std::get<0>(odomLocPoints), std::get<1>(odomLocPoints));
                auto odomM = std::get<0>(odomDetails);
                auto odomC = std::get<1>(odomDetails);
                lanelet::Point3d point1{lanelet::utils::getId(), x1, y1, 0};
                auto pProj = lanelet::geometry::project(odomLineString, point1);
                float dist = std::sqrt(std::pow((pProj.x()-x1),2)+std::pow((pProj.y()-y1),2));
                //auto y = odomM*x1+odomC;
                //auto y_diff = std::abs(std::abs(y)-std::abs(y1));
                if(dist > 0.5){
                  /*LineString ls;
                  ls.push_back(Point(x1, y1));
                  ls.push_back(Point(x2, y2));
                  allLineStrings.push_back(ls);
                  lanelet::Point3d p1{lanelet::utils::getId(), x1, y1, 0};
                  lanelet::Point3d p2{lanelet::utils::getId(), x2, y2, 0};
                  lanelet::LineString3d ls3d(lanelet::utils::getId(), {p1, p2});
                  joinLineSegments1(activeLS3d,ls3d);*/
                  joinLaneSegment(std::make_tuple(seg, m,c), std::make_tuple(odomSeg, std::get<0>(odomLineDetails), std::get<1>(odomLineDetails)));
                }
            }
            
            // Delete the corresponding active_lane_segments.
            active_lane_segments.erase(active_lane_segments.begin()+j);
          }else{
            active_lane_segments[j] = std::make_tuple(laneSegment, inactiveCount, odomPoints);
          }
        }//else close

    }//second active lane segment for loop close
    
    for(size_t k=0; k<newPointsVec.size(); k++){
      auto point = newPointsVec[k]; auto x1 = point.first; auto y1 = point.second;
      std::vector<std::pair<double, double>> laneSegment;
      laneSegment.push_back(std::make_pair(x1, y1));
      std::vector<std::pair<double, double>> odomPoints;
      odomPoints.push_back(std::make_pair(vehicle_odom_double[vehicle_odom_double.size()-1].first, vehicle_odom_double[vehicle_odom_double.size()-1].second));
      active_lane_segments.push_back(std::make_tuple(laneSegment, 0, odomPoints));

    }
  }//else close
  
}

std::tuple<std::vector<double>, std::vector<double>, LineString> FeatureExtractor::getOdomLocation(Point point1, Point point2){
  double x1 = point1.get<0>(); double y1 = point1.get<1>();
  double x2 = point2.get<0>(); double y2 = point2.get<1>();
  bool firstPointEnable = false;
  std::vector<double> xs;
  std::vector<double> ys;
  LineString linestring1;

  for(size_t i=0; i<vehicle_odom_double.size()-2; i++){
    
    if(x1 > vehicle_odom_double[i+2].first & !firstPointEnable){
      xs.push_back(vehicle_odom_double[i].first); 
      ys.push_back(vehicle_odom_double[i].second); 
      linestring1.push_back(Point(vehicle_odom_double[i].first, vehicle_odom_double[i].second));
      firstPointEnable = true;
    }
    if(firstPointEnable){
      xs.push_back(vehicle_odom_double[i].first); 
      ys.push_back(vehicle_odom_double[i].second); 
      linestring1.push_back(Point(vehicle_odom_double[i].first, vehicle_odom_double[i].second));
      if(x2 > vehicle_odom_double[i].first)
        break;
    }
  }
  
  return std::make_tuple(xs, ys, linestring1);

}

void FeatureExtractor::removeLineIntersects(){
  std::vector<size_t> eraseVec; 
  for(size_t i=0; i<allLines.size(); i++){
    auto lineSegVec = allLines[i];
    auto takeSeg = lineSegVec[0];
    auto segTuple = takeSeg.first;
    auto odomSeg = takeSeg.second;
    auto odomM = std::get<1>(odomSeg);
    auto m = std::get<1>(segTuple);
    auto slopeDiff = std::abs(odomM-m);
    auto seg1 = std::get<0>(segTuple);
    double x1 = bg::get<0, 0>(seg1); double y1 = bg::get<0, 1>(seg1);
    double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);
    float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
    for(size_t j=0; j<allLines.size(); j++){
      if(i==j)
        continue;
      auto lineSegVec2 = allLines[j];
      if(lineSegVec2.size() > 1){
         for(auto& segPair2: lineSegVec2){
          auto segTuple2 = segPair2.first;
          auto seg2 = std::get<0>(segTuple2);
          auto result = boost::geometry::intersects(seg1, seg2);
          if(result){
            eraseVec.push_back(i);
            break;
          }
        }
      }
    }
  }
  
  for(size_t i=0; i<eraseVec.size(); i++){
    ROS_INFO_STREAM("Removing line: "<<eraseVec[i]);
    allLines.erase(allLines.begin()+eraseVec[i]);
  }

}

void FeatureExtractor::joinLaneSegment(std::tuple<Segment,double,double> newSeg, std::tuple<Segment,double,double> odomSeg){
  
  //ROS_INFO_STREAM("_______________________");
  //ROS_INFO_STREAM("activeLaneSeg size: "<<activeLaneSeg.size());
  
  if(activeLaneSeg.size() == 0){
    std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>> line;
    line.push_back(std::make_pair(newSeg, odomSeg));
    activeLaneSeg.push_back(std::make_pair(line,0));
  }else{
    bool found = false;
    std::vector<std::pair<double, double>> newLineVec;
    double smallD = 100000.;
    size_t smallIndex = 0;
    std::vector<size_t> eraseVec;
    for(size_t i=0; i<activeLaneSeg.size(); i++){
      auto lineSegVec = activeLaneSeg[i].first;
      auto takeSeg = lineSegVec.back();
      auto lineSeg = takeSeg.first;
      auto seg1 = std::get<0>(lineSeg);
      auto m1 = std::get<1>(lineSeg);
      auto c1 = std::get<2>(lineSeg);
      double x1 = bg::get<0, 0>(seg1); double y1 = bg::get<0, 1>(seg1);
      double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);
      
      //Using the first principle, find y of the by inputting the x points from
      //the new segment.
      //Then project the active line seg to new point to create the new
      //extended active segment
      //Then use boost to check the extended active segment is intersect with the
      //new seg
      
      auto seg2 = std::get<0>(newSeg);
      auto m2 = std::get<1>(newSeg);
      double x3 = bg::get<0, 0>(seg2); double y3 = bg::get<0, 1>(seg2);
      double x4 = bg::get<1, 0>(seg2); double y4 = bg::get<1, 1>(seg2);
      auto slopeDiff = std::abs(m2-m1);
      auto y = m1*x4+c1;
      auto y_diff1 = std::abs(std::abs(y)-std::abs(y4));
      y = m2*x1+c1;
      auto y_diff2 = std::abs(std::abs(y)-std::abs(y1));
      float d = std::sqrt(std::pow((x4-x1),2)+std::pow((y4-y1),2));
      //ROS_INFO_STREAM("y_diff1: "<<y_diff1<<"y_diff2: "<<y_diff2<<"slopeDiff: "<<slopeDiff);
      if(slopeDiff < 0.05 && y_diff1 < 0.25 && y_diff1 < smallD){ //0.5 works
        found = true;
        smallD = y_diff1;
        smallIndex = i;
        //break;
      }/*else if(slopeDiff < 0.05 && y_diff2 < 0.25 && y_diff2 < smallD){
        found = true;
        smallD = y_diff2;
        smallIndex = i;
        //break;
      }*/
    }
    
    if(found){
      //One we found a connection between new line segement and existing active
      //lane segment we can draw the connection line between two roken line segements.
      //Sometime the line are joined incorrectly, in that case, we can say is
      //the difference between connected line and new line segment is greater
      //than a threshold, then the new line segment is not part of the current
      //active lane segment and it should be treated as new active lane
      //segment.
      auto lineSegVec = activeLaneSeg[smallIndex].first;
      auto takeSeg = lineSegVec.back();
      auto lastLineSegTuple = takeSeg.first; 
      auto odomSegTuple = takeSeg.second; 
      auto odomSeg1 = std::get<0>(odomSeg);
      auto odomSeg2 = std::get<0>(odomSegTuple);
      auto seg1 = std::get<0>(lastLineSegTuple);
      auto m1  = std::get<1>(lastLineSegTuple);
      double x1 = bg::get<0, 0>(seg1); double y1 = bg::get<0, 1>(seg1);
      double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);
      auto seg3 = std::get<0>(newSeg);
      double x5 = bg::get<0, 0>(seg3); double y5 = bg::get<0, 1>(seg3);
      double x6 = bg::get<1, 0>(seg3);double y6 = bg::get<1, 1>(seg3);
      double x3 = x2; double y3 = y2;
      double x4 = x5; double y4 = y5;
      auto dx = x4 - x3;auto dy = y4 - y3;
      if(dx != 0 && dy != 0){
        auto m2 = dy / dx;auto c2 = y3 - m2 * x3;
        auto slopeDiff = std::abs(m2-m1);
        auto odomMC = getIntermediateSlope(odomSeg1, odomSeg2);
        auto odomM = std::get<1>(odomMC);
        auto odomSlopeDiff = std::abs(odomM-m2);
        //ROS_INFO_STREAM("slopeDiff & OdomMC: "<<slopeDiff<<" "<<odomSlopeDiff);
        if(slopeDiff < 0.1 && odomSlopeDiff <= 0.01){
          //Slope diff between two lines are less than threshold, it should be
          //added to the current active lane segment.
          Segment seg2(Point(x3, y3), Point(x4, y4));
          lineSegVec.push_back(std::make_pair(std::make_tuple(seg2, m2, c2), odomMC));
          for(size_t k=0; k<activeLaneSeg.size(); k++){
            if(k == smallIndex)
              continue;
            auto lineSegVecTemp = activeLaneSeg[k].first;
            auto seg1 = std::get<0>(lineSegVecTemp[0].first);
            bool result = boost::geometry::intersects(seg1, seg2);
            if(result){
              eraseVec.push_back(k);
            }
          }
          lineSegVec.push_back(std::make_pair(newSeg, odomSeg));
          activeLaneSeg[smallIndex] = std::make_pair(lineSegVec, 0);
        }else{
          //Slope diff is greater than threshold new line segment should be
          //treated as new active lane segment.
          std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>> line;
          line.push_back(std::make_pair(newSeg,odomSeg));
          activeLaneSeg.push_back(std::make_pair(line,0));
        }
      }else{
        lineSegVec.push_back(std::make_pair(newSeg, odomSeg));
      }
      
    }else{
      std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>> line;
      line.push_back(std::make_pair(newSeg, odomSeg));
      activeLaneSeg.push_back(std::make_pair(line,0));
    }

    for(size_t i=0; i<eraseVec.size(); i++){
      activeLaneSeg.erase(activeLaneSeg.begin()+eraseVec[i]);
    }

    for(size_t i=0;i<activeLaneSeg.size();i++){
      if(i == smallIndex)
        continue;
      else{
        auto lineSegVec = activeLaneSeg[i].first;
        auto inactive = activeLaneSeg[i].second;
        inactive += 1;
        //ROS_INFO_STREAM("Inactive: "<<inactive);
        if(inactive > 20){
          //Add intermediate line inside all the lines and we can put it in
          //a seperate method
          if(lineSegVec.size() > 1){
            auto tempLineSegVec = lineSegVec;
            lineSegVec.clear();
            lineSegVec = joinEachSegInALine(tempLineSegVec);
          }
          auto line = segPairsToLineString(lineSegVec);
          allLineStrings.push_back(line);
          //joinLinesFurther(lineSegVec);
          activeLaneSeg.erase(activeLaneSeg.begin()+i);
        }else{
          activeLaneSeg[i] = std::make_pair(lineSegVec, inactive);
        }
      }
    }// second active lane seg for loop close 
  }//main else close
 
}

void FeatureExtractor::joinLinesFurther(std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>> newLine){
  //ROS_INFO_STREAM("+++++++++++++++++++++++++");
  //ROS_INFO_STREAM("activeLaneSeg2 size: "<<activeLaneSeg2.size());
  if(activeLaneSeg2.size() == 0){
    activeLaneSeg2.push_back(std::make_pair(newLine,0));
  }else{
    bool found = false;
    std::vector<std::pair<double, double>> newLineVec;
    double smallD = 100000.;
    size_t smallIndex = 0;
    std::vector<size_t> eraseVec;
    for(size_t i=0; i<activeLaneSeg2.size(); i++){
      auto lineSegVec = activeLaneSeg2[i].first;
      auto takeSeg = lineSegVec.back();
      auto lineSeg = takeSeg.first;
      auto seg1 = std::get<0>(lineSeg);
      auto m1 = std::get<1>(lineSeg);
      auto c1 = std::get<2>(lineSeg);
      double x1 = bg::get<0, 0>(seg1); double y1 = bg::get<0, 1>(seg1);
      double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);
      auto seg2pair = newLine.front();
      auto seg2 = std::get<0>(seg2pair.first);
      auto m2 = std::get<1>(seg2pair.first);
      double x3 = bg::get<0, 0>(seg2); double y3 = bg::get<0, 1>(seg2);
      double x4 = bg::get<1, 0>(seg2); double y4 = bg::get<1, 1>(seg2);
      auto slopeDiff = std::abs(m2-m1);
      auto y = m1*x4+c1;
      auto y_diff1 = std::abs(std::abs(y)-std::abs(y4));
      y = m2*x1+c1;
      auto y_diff2 = std::abs(std::abs(y)-std::abs(y1));
      float d = std::sqrt(std::pow((x4-x1),2)+std::pow((y4-y1),2));
      if(slopeDiff < 0.07 && y_diff1 < 0.25 && y_diff1 < smallD){ //0.5 works
        found = true;
        smallD = y_diff1;
        smallIndex = i;
        break;
      }
    }  
    bool proceedSmallIndex = true;
    if(found){
      auto lineSegVec = activeLaneSeg2[smallIndex].first;
      auto takeSeg = lineSegVec.back();
      auto lastLineSegTuple = takeSeg.first; 
      auto odomSegTuple = takeSeg.second; 
      //auto odomM = std::get<1>(odomSegTuple);
      auto odomSeg1 = std::get<0>(odomSegTuple);
      auto seg1 = std::get<0>(lastLineSegTuple);
      auto m1  = std::get<1>(lastLineSegTuple);
      double x1 = bg::get<0, 0>(seg1); double y1 = bg::get<0, 1>(seg1);
      double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);
      
      auto seg3pair = newLine.front();
      auto seg3 = std::get<0>(seg3pair.first);
      //auto odomM = std::get<1>(seg3pair.second);
      auto odomSeg2 = std::get<0>(seg3pair.second);
      double x5 = bg::get<0, 0>(seg3); double y5 = bg::get<0, 1>(seg3);
      double x6 = bg::get<1, 0>(seg3);double y6 = bg::get<1, 1>(seg3);
      double x3 = x2; double y3 = y2;
      double x4 = x5; double y4 = y5;
      
      float d = std::sqrt(std::pow((x4-x3),2)+std::pow((y4-y3),2));
      auto dx = x4 - x3;auto dy = y4 - y3;
      if(dx != 0 && dy != 0){
        auto m2 = dy / dx;auto c2 = y3 - m2 * x3;
        auto slopeDiff = std::abs(m2-m1);
        auto odomMC = getIntermediateSlope(odomSeg1, odomSeg2);
        auto odomM = std::get<1>(odomMC);
        auto odomSlopeDiff = std::abs(odomM-m2);
        //ROS_INFO_STREAM("slopeDiff && second odomM: "<<slopeDiff<<" "<<odomSlopeDiff);
        if(slopeDiff < 0.1 && odomSlopeDiff < 0.05){ //0.2 and 0.1 works
          if(d < 50){
            auto check = checkIntersectionOdomAndConnectionLine(std::make_pair(x3, y3), std::make_pair(x4, y4), std::get<0>(odomMC));
            //Insert only if there is no intersection with odom 
            if(!check){ 
              Segment seg2(Point(x3, y3), Point(x4, y4));
              lineSegVec.push_back(std::make_pair(std::make_tuple(seg2, m2, c2), odomMC));
              for(auto eachSegPair: newLine){
                lineSegVec.push_back(eachSegPair);
              }
              activeLaneSeg2[smallIndex] = std::make_pair(lineSegVec, 0);
            }else{
              //Intersection with odom found.
              activeLaneSeg2.push_back(std::make_pair(newLine,0));
            }
          }else{
            activeLaneSeg2.push_back(std::make_pair(newLine,0));
          }
        }else{

          /*float d1 = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
          if(d1 < 5){
            proceedSmallIndex = false;
            auto line = segPairsToLineString(lineSegVec);
            allLineStrings.push_back(line);
          }else*/
          
          activeLaneSeg2.push_back(std::make_pair(newLine,0));
        }
      }else{
        activeLaneSeg2.push_back(std::make_pair(newLine,0));
      }
      
    }else{
      activeLaneSeg2.push_back(std::make_pair(newLine,0));
    }

    for(size_t i=0;i<activeLaneSeg2.size();i++){
      if(i == smallIndex && proceedSmallIndex)
        continue;
      else{
        auto lineSegVec = activeLaneSeg2[i].first;
        auto inactive = activeLaneSeg2[i].second;
        inactive += 1;
        //ROS_INFO_STREAM("Inactive: "<<inactive);
        if(inactive > 10){
          //Add intermediate line inside all the lines and we can put it in
          //a seperate method
          if(lineSegVec.size() > 1){
            auto tempLineSegVec = lineSegVec;
            lineSegVec.clear();
            lineSegVec = joinEachSegInALine(tempLineSegVec);
          }
          /*auto line = segPairsToLineString(lineSegVec);
          allLineStrings.push_back(line);*/
          //removeNoiseLines(5.);
          joinLinesFurther2(lineSegVec);
          activeLaneSeg2.erase(activeLaneSeg2.begin()+i);
        }else{
          activeLaneSeg2[i] = std::make_pair(lineSegVec, inactive);
        }
      }
    }// second active lane seg for loop close
  }//main else close
}

void FeatureExtractor::joinLinesFurther2(std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>> newLine){
  //ROS_INFO_STREAM("+++++++++++++++++++++++++");
  //ROS_INFO_STREAM("activeLaneSeg3 size: "<<activeLaneSeg3.size());
  if(activeLaneSeg3.size() == 0){
    activeLaneSeg3.push_back(std::make_pair(newLine,0));
  }else{
    bool found = false;
    std::vector<std::pair<double, double>> newLineVec;
    double smallD = 100000.;
    size_t smallIndex = 0;
    for(size_t i=0; i<activeLaneSeg3.size(); i++){
      auto lineSegVec = activeLaneSeg3[i].first;
      
      // Find average m&c using the last two segment in a line 
      int count = 0;
      double m1 = 0.;
      double c1 = 0.;
      for(auto& takeSeg: lineSegVec){
        if(count == 3)
          break;
        m1 += std::get<1>(takeSeg.first);
        c1 += std::get<2>(takeSeg.first);
        count++;
      }
      m1 /= count;
      c1 /= count;
      
      auto takeSeg = lineSegVec.back();
      auto lineSeg = takeSeg.first;
      auto seg1 = std::get<0>(lineSeg);
      /*auto m1 = std::get<1>(lineSeg);
      auto c1 = std::get<2>(lineSeg);*/
      double x1 = bg::get<0, 0>(seg1); double y1 = bg::get<0, 1>(seg1);
      double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);
      auto seg2pair = newLine.front();
      auto seg2 = std::get<0>(seg2pair.first);
      /*auto m2 = std::get<1>(seg2pair.first);
      auto c2 = std::get<2>(seg2pair.first);*/
      count = 0;
      double m2 = 0;
      double c2 = 0;
      for(auto takeSeg: newLine){
        if(count == 3)
          break;
        m2 += std::get<1>(takeSeg.first);
        c2 += std::get<2>(takeSeg.first);
        count++;
      }
      
      m2 /= count;
      c2 /= count;

      double x3 = bg::get<0, 0>(seg2); double y3 = bg::get<0, 1>(seg2);
      double x4 = bg::get<1, 0>(seg2); double y4 = bg::get<1, 1>(seg2);
      auto slopeDiff = std::abs(m2-m1);
      /*auto y = m1*x4+c1;
      auto y_diff1 = std::abs(std::abs(y)-std::abs(y4));
      y = m2*x1+c1;
      auto y_diff2 = std::abs(std::abs(y)-std::abs(y1));*/
      auto y = m1*x4+c1;
      auto y_diff1 = std::abs(std::abs(y)-std::abs(y4));
      float d = std::sqrt(std::pow((x4-x1),2)+std::pow((y4-y1),2));
      if(slopeDiff < 0.1 && y_diff1 < 0.25 && y_diff1 < smallD){ //0.5 works
        found = true;
        smallD = y_diff1;
        smallIndex = i;
        break;
      }
    }
    if(found){
      auto lineSegVec = activeLaneSeg3[smallIndex].first;
      auto takeSeg = lineSegVec.back();
      auto lastLineSegTuple = takeSeg.first;
      auto odomSegTuple = takeSeg.second;
      //auto odomM = std::get<1>(odomSegTuple);
      auto odomSeg1 = std::get<0>(odomSegTuple);
      auto seg1 = std::get<0>(lastLineSegTuple);
      auto m1  = std::get<1>(lastLineSegTuple);
      double x1 = bg::get<0, 0>(seg1); double y1 = bg::get<0, 1>(seg1);
      double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);

      auto seg3pair = newLine.front();
      auto seg3 = std::get<0>(seg3pair.first);
      //auto odomM = std::get<1>(seg3pair.second);
      auto odomSeg2 = std::get<0>(seg3pair.second);
      double x5 = bg::get<0, 0>(seg3); double y5 = bg::get<0, 1>(seg3);
      double x6 = bg::get<1, 0>(seg3);double y6 = bg::get<1, 1>(seg3);
      double x3 = x1; double y3 = y1;
      double x4 = x6; double y4 = y6;
      auto dx = x4 - x3;auto dy = y4 - y3;
      if(dx != 0 && dy != 0){
        auto m2 = dy / dx;auto c2 = y3 - m2 * x3;
        auto slopeDiff = std::abs(m2-m1);
        auto odomMC = getIntermediateSlope(odomSeg1, odomSeg2);
        auto odomM = std::get<1>(odomMC);
        auto odomSlopeDiff = std::abs(odomM-m2);
        //ROS_INFO_STREAM("slopeDiff && second odomM: "<<slopeDiff<<" "<<odomSlopeDiff);
        float d = std::sqrt(std::pow((x4-x3),2)+std::pow((y4-y3),2));
        if(slopeDiff < 0.1 && odomSlopeDiff < 0.05){ //0.2 and 0.1 works
          if(d < 50){
            auto check = checkIntersectionOdomAndConnectionLine(std::make_pair(x3, y3), std::make_pair(x4, y4), std::get<0>(odomMC));
            //Insert only if there is no intersection with odom 
            if(!check){ 
              Segment seg2(Point(x3, y3), Point(x4, y4));
              lineSegVec.push_back(std::make_pair(std::make_tuple(seg2, m2, c2), odomMC));
              for(auto eachSegPair: newLine){
                lineSegVec.push_back(eachSegPair);
              }
              activeLaneSeg3[smallIndex] = std::make_pair(lineSegVec, 0);
            }else{
              //Intersection found with odom
              activeLaneSeg3.push_back(std::make_pair(newLine,0));
            }
          }else{
            activeLaneSeg3.push_back(std::make_pair(newLine,0));
          }
        }else{
          activeLaneSeg3.push_back(std::make_pair(newLine,0));
        }
      }else{
        for(auto eachSegPair: newLine){
          lineSegVec.push_back(eachSegPair);
        }
      }

    }else{
      activeLaneSeg3.push_back(std::make_pair(newLine,0));
    }

    for(size_t i=0;i<activeLaneSeg3.size();i++){
      if(i == smallIndex)
        continue;
      else{
        auto lineSegVec = activeLaneSeg3[i].first;
        auto inactive = activeLaneSeg3[i].second;
        inactive += 1;
        //ROS_INFO_STREAM("Inactive: "<<inactive);
        if(inactive > 5){
          //Add intermediate line inside all the lines and we can put it in
          //a seperate method
          if(lineSegVec.size() > 1){
            auto tempLineSegVec = lineSegVec;
            lineSegVec.clear();
            lineSegVec = joinEachSegInALine(tempLineSegVec);
          }
          
          removeNoiseLines(5.);
          /*auto line = segPairsToLineString(lineSegVec);
          allLineStrings.push_back(line);*/
          joinLinesFurther3(lineSegVec);
          activeLaneSeg3.erase(activeLaneSeg3.begin()+i);
        }else{
          activeLaneSeg3[i] = std::make_pair(lineSegVec, inactive);
        }
      }
    }// second active lane seg for loop close
  }//main else close

}

void FeatureExtractor::joinLinesFurther3(std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>> newLine){
  //ROS_INFO_STREAM("+++++++++++++++++++++++++");
  //ROS_INFO_STREAM("activeLaneSeg3 size: "<<activeLaneSeg3.size());
  if(activeLaneSeg4.size() == 0){
    activeLaneSeg4.push_back(std::make_pair(newLine,0));
  }else{
    bool found = false;
    std::vector<std::pair<double, double>> newLineVec;
    double smallD = 100000.;
    size_t smallIndex = 0;
    for(size_t i=0; i<activeLaneSeg4.size(); i++){
      auto lineSegVec = activeLaneSeg4[i].first;
      
      // Find average m&c using the last two segment in a line 
      int count = 0;
      double m1 = 0.;
      double c1 = 0.;
      for(auto& takeSeg: lineSegVec){
        if(count == 3)
          break;
        m1 += std::get<1>(takeSeg.first);
        c1 += std::get<2>(takeSeg.first);
        count++;
      }
      m1 /= count;
      c1 /= count;
      
      auto takeSeg = lineSegVec.back();
      auto lineSeg = takeSeg.first;
      auto seg1 = std::get<0>(lineSeg);
      /*auto m1 = std::get<1>(lineSeg);
      auto c1 = std::get<2>(lineSeg);*/
      double x1 = bg::get<0, 0>(seg1); double y1 = bg::get<0, 1>(seg1);
      double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);
      auto seg2pair = newLine.front();
      auto seg2 = std::get<0>(seg2pair.first);
      /*auto m2 = std::get<1>(seg2pair.first);
      auto c2 = std::get<2>(seg2pair.first);*/
      count = 0;
      double m2 = 0;
      double c2 = 0;
      for(auto takeSeg: newLine){
        if(count == 3)
          break;
        m2 += std::get<1>(takeSeg.first);
        c2 += std::get<2>(takeSeg.first);
        count++;
      }
      
      m2 /= count;
      c2 /= count;

      double x3 = bg::get<0, 0>(seg2); double y3 = bg::get<0, 1>(seg2);
      double x4 = bg::get<1, 0>(seg2); double y4 = bg::get<1, 1>(seg2);
      auto slopeDiff = std::abs(m2-m1);
      /*auto y = m1*x4+c1;
      auto y_diff1 = std::abs(std::abs(y)-std::abs(y4));
      y = m2*x1+c1;
      auto y_diff2 = std::abs(std::abs(y)-std::abs(y1));*/
      auto y = m1*x4+c1;
      auto y_diff1 = std::abs(std::abs(y)-std::abs(y4));
      float d = std::sqrt(std::pow((x4-x1),2)+std::pow((y4-y1),2));
      if(slopeDiff < 0.2 && y_diff1 < 0.5 && y_diff1 < smallD){ //0.05 works
        found = true;
        smallD = y_diff1;
        smallIndex = i;
        //break;
      }
    }
    if(found){
      auto lineSegVec = activeLaneSeg4[smallIndex].first;
      auto takeSeg = lineSegVec.back();
      auto lastLineSegTuple = takeSeg.first;
      auto odomSegTuple = takeSeg.second;
      //auto odomM = std::get<1>(odomSegTuple);
      auto odomSeg1 = std::get<0>(odomSegTuple);
      auto seg1 = std::get<0>(lastLineSegTuple);
      auto m1  = std::get<1>(lastLineSegTuple);
      double x1 = bg::get<0, 0>(seg1); double y1 = bg::get<0, 1>(seg1);
      double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);

      auto seg3pair = newLine.front();
      auto seg3 = std::get<0>(seg3pair.first);
      //auto odomM = std::get<1>(seg3pair.second);
      auto odomSeg2 = std::get<0>(seg3pair.second);
      double x5 = bg::get<0, 0>(seg3); double y5 = bg::get<0, 1>(seg3);
      double x6 = bg::get<1, 0>(seg3);double y6 = bg::get<1, 1>(seg3);
      double x3 = x1; double y3 = y1;
      double x4 = x6; double y4 = y6;
      auto dx = x4 - x3;auto dy = y4 - y3;
      if(dx != 0 && dy != 0){
        auto m2 = dy / dx;auto c2 = y3 - m2 * x3;
        auto slopeDiff = std::abs(m2-m1);
        auto odomMC = getIntermediateSlope(odomSeg1, odomSeg2);
        auto odomM = std::get<1>(odomMC);
        auto odomSlopeDiff = std::abs(odomM-m2);
        //ROS_INFO_STREAM("slopeDiff && second odomM: "<<slopeDiff<<" "<<odomSlopeDiff);
        float d = std::sqrt(std::pow((x4-x3),2)+std::pow((y4-y3),2));
        if(slopeDiff < 0.1 && odomSlopeDiff < 0.05){ //0.2 and 0.1 works most of the cases
          if(d < 50){
            Segment seg2(Point(x3, y3), Point(x4, y4));
            lineSegVec.push_back(std::make_pair(std::make_tuple(seg2, m2, c2), odomMC));
            for(auto eachSegPair: newLine){
              lineSegVec.push_back(eachSegPair);
            }
            activeLaneSeg4[smallIndex] = std::make_pair(lineSegVec, 0);
          }else{
            activeLaneSeg4.push_back(std::make_pair(newLine,0));
          }
        }else{
          activeLaneSeg4.push_back(std::make_pair(newLine,0));
        }
      }else{
        for(auto eachSegPair: newLine){
          lineSegVec.push_back(eachSegPair);
        }
      }

    }else{
      activeLaneSeg4.push_back(std::make_pair(newLine,0));
    }

    for(size_t i=0;i<activeLaneSeg4.size();i++){
      if(i == smallIndex)
        continue;
      else{
        auto lineSegVec = activeLaneSeg4[i].first;
        auto inactive = activeLaneSeg4[i].second;
        inactive += 1;
        //ROS_INFO_STREAM("Inactive: "<<inactive);
        if(inactive > 5){
          //Add intermediate line inside all the lines and we can put it in
          //a seperate method
          if(lineSegVec.size() > 1){
            auto tempLineSegVec = lineSegVec;
            lineSegVec.clear();
            lineSegVec = joinEachSegInALine(tempLineSegVec);
          }
          allLineStrings.push_back(segPairsToLineString(lineSegVec));
          activeLaneSeg4.erase(activeLaneSeg4.begin()+i);
        }else{
          activeLaneSeg4[i] = std::make_pair(lineSegVec, inactive);
        }
      }
    }// second active lane seg for loop close
  }//main else close

}

LineString FeatureExtractor::segPairsToLineString(std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>> newLine){
  
  std::vector<std::vector<double>> sortedVec1;
  LineString ls1;
  for(auto& linePair: newLine){
    auto segTuple = linePair.first;
    auto seg1 = std::get<0>(segTuple);
    double x1 = bg::get<0, 0>(seg1); double y1 = bg::get<0, 1>(seg1);
    double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);
    std::vector<double> row;
    row.push_back(x1);
    row.push_back(y1);
    sortedVec1.push_back(row);
    
    row.clear();
    row.push_back(x2);
    row.push_back(y2);
    sortedVec1.push_back(row);

  }
  std::sort(sortedVec1.begin(), sortedVec1.end(), [](const std::vector<double>& a, const std::vector<double>& b) {return a[0] > b[0];});
  for(auto& vec: sortedVec1){
    ls1.push_back(Point(vec[0], vec[1])); 
  }

  return ls1;
}

std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>> FeatureExtractor::joinEachSegInALine(std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>> tempLineSegVec){
  std::vector<std::pair<std::tuple<Segment,double,double>, std::tuple<Segment,double,double>>> lineSegVec;
  for(size_t k=0; k<tempLineSegVec.size()-1; k++){
    auto takeSeg1 = tempLineSegVec[k];
    auto firstSegTuple = takeSeg1.first; 
    auto seg1 = std::get<0>(firstSegTuple);
    auto odomSeg1 = std::get<0>(takeSeg1.second);
    lineSegVec.push_back(takeSeg1);
    auto takeSeg3 = tempLineSegVec[k+1];
    auto secondSegTuple = takeSeg3.first; 
    auto seg3 = std::get<0>(secondSegTuple);
    auto odomSeg3 = std::get<0>(takeSeg3.second);
    double x1 = bg::get<0, 0>(seg1); double y1 = bg::get<0, 1>(seg1);
    double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);
    double x5 = bg::get<0, 0>(seg3); double y5 = bg::get<0, 1>(seg3);
    double x6 = bg::get<1, 0>(seg3);double y6 = bg::get<1, 1>(seg3);
    double x3 = x2; double y3 = y2;
    double x4 = x5; double y4 = y5;
    auto dx = x4 - x3;auto dy = y4 - y3;
    if(dx != 0 && dy != 0){
      auto m2 = dy / dx;auto c2 = y3 - m2 * x3;
      auto odomMC = getIntermediateSlope(odomSeg1, odomSeg3);
      Segment seg2(Point(x3, y3), Point(x4, y4));
      lineSegVec.push_back(std::make_pair(std::make_tuple(seg2, m2, c2), odomMC));
    }
    lineSegVec.push_back(takeSeg3);
  }

  return lineSegVec;
}

void FeatureExtractor::removeTheLineinConnectedPath(){
  std::vector<size_t> eraseVec;
  for(size_t i=0; i<allLines.size(); i++){
    auto lineSegVec = allLines[i];
    LineString linestring1;
    for(size_t k=0; k<lineSegVec.size(); k++){
      auto seg1 = std::get<0>(lineSegVec[k].first);
      double x1 = bg::get<0, 0>(seg1);double y1 = bg::get<0, 1>(seg1);
      double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);
      linestring1.push_back(Point(x1-0.2,y1-0.2)); 
      linestring1.push_back(Point(x2+0.2,y2+0.2)); 
    }
    auto seg21 = std::get<0>(lineSegVec.front().first);
    auto seg22 = std::get<0>(lineSegVec.back().first);
    double x3 = bg::get<0, 0>(seg21);double y3 = bg::get<0, 1>(seg21);
    double x4 = bg::get<1, 0>(seg22);double y4 = bg::get<1, 1>(seg22);
    Segment segCheck1(Point(x3, y3), Point(x4, y4));

    for(size_t j=0; j<allLines.size(); j++){
      if (std::find(eraseVec.begin(), eraseVec.end(), j) != eraseVec.end() || i==j)
          continue;
      auto lineSegVec2 = allLines[j];
      LineString linestring2;
      for(size_t k=0; k<lineSegVec2.size(); k++){
        auto seg1 = std::get<0>(lineSegVec2[k].first);
        double x1 = bg::get<0, 0>(seg1);double y1 = bg::get<0, 1>(seg1);
        double x2 = bg::get<1, 0>(seg1);double y2 = bg::get<1, 1>(seg1);
        linestring2.push_back(Point(x1-0.2,y1-0.2)); 
        linestring2.push_back(Point(x2+0.2,y2+0.2)); 
      }
      seg21 = std::get<0>(lineSegVec2.front().first);
      seg22 = std::get<0>(lineSegVec2.back().first);
      x3 = bg::get<0, 0>(seg21);double y3 = bg::get<0, 1>(seg21);
      x4 = bg::get<1, 0>(seg22);double y4 = bg::get<1, 1>(seg22);
      Segment segCheck2(Point(x3, y3), Point(x4, y4));
      MultiLineString intersection;
      bg::intersection(linestring1, linestring2, intersection);
      
      if(intersection.size() > 1){
        auto d1 = bg::length(segCheck1);
        auto d2 = bg::length(segCheck2);
        if(d1 > 20 && d2 > 20)
          continue;
        else{
          auto selectedIndex = j;
          if(d1 < d2)
            selectedIndex = i;
          ROS_INFO_STREAM("Boost intersection found: "<<bg::length(segCheck1)<<" "<<bg::length(segCheck2));
          eraseVec.push_back(selectedIndex);
          break;
        }
      } 
    }
  }
  
  for(size_t i=0; i<eraseVec.size(); i++){
   allLines.erase(allLines.begin()+eraseVec[i]);
  }

}

std::tuple<Segment, double, double> FeatureExtractor::getIntermediateSlope(Segment seg1, Segment seg2){
  double x1 = bg::get<1, 0>(seg1); double y1 = bg::get<1, 1>(seg1);
  double x2 = bg::get<0, 0>(seg2); double y2 = bg::get<0, 1>(seg2);
  auto dx = x2 - x1;auto dy = y2 - y1;
  auto m = 10000.; auto c = 10000.; 
  if(dx != 0 && dy != 0){
    m = dy / dx;c = y1 - m * x1;
  }
  Segment seg(Point(x1,y1), Point(x2, y2));

  return std::make_tuple(seg, m, c);

}

void FeatureExtractor::removeNoiseLines(double distT){
  for(size_t i=0; i<allLineStrings.size(); i++){
    auto ls3d = getlaneletLineStringFromBoostLineString(allLineStrings[i]);
    auto d = lanelet::geometry::length(lanelet::utils::toHybrid(ls3d));
    if(d < distT){
      allLineStrings.erase(allLineStrings.begin()+i);
    }
  }
}

// call whenever receive a pointcloud - spit out new filtered version
void
FeatureExtractor::SegmentPointCloud_intensity(sensor_msgs::PointCloud2::ConstPtr pointcloud_msg,
                                              std::map<std::pair<int, int>, double> &intensity_topic) {

  try {

    // transform from the world reference to the base link
    auto world_transform = transformer_->lookupTransform(projection_frame,
                                                         std::string("base_link"), pointcloud_msg->header.stamp);

   
    // transform from the base_link_horizon to the lidar reference frame -
    //  this will correct for the pitch and roll of the platform
    //auto platform_transform = transformer_->lookupTransform(std::string("base_link_horizon"),
    auto platform_transform = transformer_->lookupTransform(std::string("base_link"),
                                                            std::string(pointcloud_msg->header.frame_id),
                                                            pointcloud_msg->header.stamp);

    // Transform pointcloud into base_link_horizon frame
    Eigen::Quaternionf platform_rotation(platform_transform.transform.rotation.w,
                                         platform_transform.transform.rotation.x,
                                         platform_transform.transform.rotation.y,
                                         platform_transform.transform.rotation.z);

    Eigen::Vector3f platform_origin(platform_transform.transform.translation.x,
                                    platform_transform.transform.translation.y,
                                    platform_transform.transform.translation.z);

    Eigen::Quaternionf world_rotation(world_transform.transform.rotation.w,
                                      world_transform.transform.rotation.x,
                                      world_transform.transform.rotation.y,
                                      world_transform.transform.rotation.z);

    Eigen::Vector3f world_origin(world_transform.transform.translation.x,
                                 world_transform.transform.translation.y,
                                 world_transform.transform.translation.z);


    pcl::PointCloud<pcl::PointXYZIR>::Ptr input_pointcloud_unrotated(new pcl::PointCloud<pcl::PointXYZIR>);
    pcl::PointCloud<pcl::PointXYZIR>::Ptr input_pointcloud_box_filter(new pcl::PointCloud<pcl::PointXYZIR>);
    pcl::PointCloud<pcl::PointXYZIR>::Ptr input_pointcloud(new pcl::PointCloud<pcl::PointXYZIR>);


    // the original unrotated point cloud
    pcl::fromROSMsg(*pointcloud_msg, *input_pointcloud_unrotated);

    pcl::PointCloud<pcl::PointXYZIR>::Ptr input_downsampled = Selector(input_pointcloud_unrotated,
                                                                       30.,
                                                                       ring_filter,
                                                                       rings_included);

    float range_ = 50;
    // Bounding box filter
    pcl::CropBox<pcl::PointXYZIR> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(-range_, -range_, -range_, range_));
    boxFilter.setMax(Eigen::Vector4f(range_, range_, range_, range_));
    boxFilter.setInputCloud(input_downsampled);
    boxFilter.filter(*input_pointcloud_box_filter);
    pcl::transformPointCloud(*input_pointcloud_box_filter, *input_pointcloud, platform_origin, platform_rotation);
    
    pcl::PointCloud<pcl::PointXYZIR>::Ptr input_pointcloud_bl(new pcl::PointCloud<pcl::PointXYZIR>);
    pcl::copyPointCloud(*input_pointcloud_box_filter, *input_pointcloud_bl);
    pcl::transformPointCloud(*input_pointcloud, *input_pointcloud, world_origin, world_rotation);

    int x_index = (world_origin[0] * 100.) / cm_resolution;
    int y_index = (world_origin[1] * 100.) / cm_resolution;
    vehicle_odom.push_back(std::make_pair(-1. * y_index, x_index));
    vehicle_odom_double.push_back(std::make_pair(world_origin[0], world_origin[1]));
    lanelet::Point3d odomLLPoint{lanelet::utils::getId(), world_origin[0], world_origin[1], 0};
    odomLineString.push_back(odomLLPoint);
    double z_min = 100000000.;
    double z_max  = -100000000.;
       
    if(input_pointcloud_bl->points.size() > 0){
      auto extractedPC = extractEdges(input_pointcloud_bl, pointcloud_msg->header.stamp.sec, pointcloud_msg->header.stamp.nsec);
      
      if(extractedPC.points.size() > 0){ 
        // Filtering on baselink
        auto lanePC = pointCloudFilter(extractedPC);
        
        // Transfroming to odom frame
        pcl::transformPointCloud(lanePC, lanePC, world_origin, world_rotation);
        
        // Construct lane segement
        constructLaneSegments(lanePC); 
        
        for(size_t i=0; i<lanePC.points.size(); i++){		
          auto x = lanePC.points[i].x;
          auto y = lanePC.points[i].y;
          int x_index = (x * 100.) / cm_resolution;
          int y_index = (y * 100.) / cm_resolution;
          lane_points.push_back(std::make_pair(-1. * y_index, x_index));
          lane_odom.push_back(std::make_pair(x, y));
        }
        seq_count += 1;
        
        
      }
    }
    
    // put each point into the intensity map
    for (size_t i = 0; i < input_pointcloud->points.size(); ++i) {
      int x_index = (input_pointcloud->points[i].x * 100.) / cm_resolution;
      int y_index = (input_pointcloud->points[i].y * 100.) / cm_resolution;

      if (fabs(x_index) > 1e7 || fabs(y_index) > 1e7) {
        continue;
      }
     
      intensity_topic[std::make_pair(-1. * y_index,
                                     x_index)] = input_pointcloud->points[i].intensity; // std::map<std::pair<int,int>, double>
    }


  } catch (const std::exception &e) { // reference to the base of a polymorphic object
    ROS_ERROR_STREAM(e.what()); // information from length_error printed
  }

}

// call whenever receive a pointcloud - spit out new filtered version
void
FeatureExtractor::SegmentPointCloud_label(sensor_msgs::PointCloud2::ConstPtr pointcloud_msg,
                                          std::map<std::pair<int, int>, double> &intensity_topic) {

  /*
  // convert ROS message to PCL object
  pcl::PointCloud<pcl::PointXYZIRL>::Ptr input_pointcloud_unrotated(new pcl::PointCloud<pcl::PointXYZIRL>);
  pcl::PointCloud<pcl::PointXYZIRL>::Ptr input_pointcloud(new pcl::PointCloud<pcl::PointXYZIRL>);

  pcl::PointCloud<pcl::PointXYZIRL>::Ptr intermediate(new pcl::PointCloud<pcl::PointXYZIRL>);

  // the original unrotated point cloud
  pcl::fromROSMsg(*pointcloud_msg, *input_pointcloud_unrotated);


  pcl::PointCloud<pcl::PointXYZIRL>::Ptr input_downsampled = Selector_label(input_pointcloud_unrotated,
                                                                            30.,
                                                                            ring_filter,
                                                                            rings_included);

  // transform the point cloud to compensate for the vehicle pitch and roll
  tf::quaternionTFToEigen(imu_rotation, eigen_q);
  tf::quaternionTFToEigen(odom_rotation, eigen_odom);

  pcl::transformPointCloud(*input_downsampled, *input_pointcloud, odom_vector, eigen_odom);

  // put each point into the intensity map
  for (size_t i = 0; i < input_pointcloud->points.size(); ++i) {
    int x_index = (input_pointcloud->points[i].x * 100.) / cm_resolution;
    int y_index = (input_pointcloud->points[i].y * 100.) / cm_resolution;
    intensity_topic[std::make_pair(-1. * y_index,
                                   x_index)] = input_pointcloud->points[i].label; // std::map<std::pair<int,int>, double>
  }
   */
}

