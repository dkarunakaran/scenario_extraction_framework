#include <h264_bag_playback/h264_bag_playback.hpp>
#include <dataset_tools/run_pipeline.hpp>
#include <dataset_tools/point_cloud_features_pipeline.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <dataset_msgs/DatasetEvent.h>
#include <sensor_msgs/NavSatFix.h>
#include <custom_point_types/point_xyzir.h>
#include <chrono>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "json.hpp"
#include <tf/tf.h>
#include <ibeo_object_msg/IbeoObject.h> 
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/io_handlers/Factory.h>
#include <lanelet2_io/io_handlers/Writer.h>
#include <lanelet2_projection/UTM.h>
#include <tf2_ros/transform_listener.h>
#include "helper_functions.hpp"
#include <unistd.h>

namespace bg = boost::geometry;
using json = nlohmann::json;

class Extraction : public dataset_toolkit::h264_bag_playback {

  public:
    int previous_percentage;
    // variables to estimate the time remaining to perform the projection
    uint32_t current_message_number;
    std::chrono::steady_clock::time_point start_time, end_time;
    std::string odometry_topic; std::string objects_topic;
    std::string bag_file;
    std::string centerline_json_file;
    std::string scenario_json_file;
    std::string cars_frenet_json_file;
    std::string ego_frenet_json_file;
    bool resume;
    bool storeDataInitially;
    std::string lanelet_file; lanelet::LaneletMapPtr map;
    std::vector<int> objectClassVec;
    double frenetS;
    std::vector<std::pair<int, std::vector<json>>> frenetJson;
    std::vector<std::pair<int, std::vector<json>>> frenetJsonOtherEgoRef;
    lanelet::LineString3d roadCenterLine;
    lanelet::LineString3d egoCenterLine;
    std::vector<json> frenetJsonEgo;
    long int egoDataCount;
    long int positiveDir;
    long int negativeDir;
    std::vector<int> cutInScenarioNoDetect;
    std::vector<json> cutInScenarios;
    std::vector<int> cutInScenarioCar;
    ros::Publisher finishPub;
    ros::NodeHandle nh;
    ros::Subscriber sub;
    std_msgs::String pubMsg;
    std::vector<uint32_t> egoSecVec;
    std::vector<std::tuple<int, uint32_t, int>> objSecVec; 
    std::vector<int> cutOutScenarioNoDetect;
    std::vector<json> cutOutScenarios;
    std::vector<int> cutOutScenarioCar;
    std::vector<json> laneFollowingScenarios;
    std::vector<int> laneFollowingCar;
    int currentEgoLaneNo;
    double currentEgoSpeed;
    bool standalone_run;
    
    Extraction() : h264_bag_playback() {
      previous_percentage = -1;
      current_message_number = 0;
      private_nh.getParam("objects", objects_topic);
      private_nh.getParam("odometry", odometry_topic);
      private_nh.getParam("bag_file", bag_file);
      private_nh.getParam("resume", resume);
      private_nh.getParam("centerline_json_file", centerline_json_file);
      private_nh.getParam("lanelet_file", lanelet_file);
      private_nh.getParam("scenario_json_file", scenario_json_file);
      private_nh.getParam("cars_frenet_json_file", cars_frenet_json_file);
      private_nh.getParam("ego_frenet_json_file", ego_frenet_json_file);
      private_nh.getParam("standalone_run", standalone_run);
      objectClassVec.push_back(4); //bike
      objectClassVec.push_back(5); //car
      objectClassVec.push_back(6); //truck
      objectClassVec.push_back(15); //motorbike
      frenetS = 0;
      egoDataCount = 0;
      positiveDir = 0;
      negativeDir = 0;
      finishPub = nh.advertise<std_msgs::String>("finish_extraction", 1000);
      sub = nh.subscribe<std_msgs::String> ("finish_map_generation", 1, &Extraction::begin, this);
      lanelet::projection::UtmProjector projector(lanelet::Origin({0, 0}));  
      map = lanelet::load(lanelet_file, projector);
      currentEgoLaneNo = 0;
      currentEgoSpeed = 0.;
    }

    void begin(const std_msgs::String::ConstPtr& message){
      ROS_INFO_STREAM("Resume: "<<resume);
      ROS_INFO_STREAM("Map is loading");
      lanelet::projection::UtmProjector projector(lanelet::Origin({0, 0}));  
      map = lanelet::load(lanelet_file, projector);
      sleep(2);
      ROS_INFO_STREAM("Map loaded");
      if(resume){
        ROS_INFO_STREAM("Data is loading from JSON files...");
        loadData();
      }else{
        ROS_INFO_STREAM("Data is loading from Bag file...");
        storeDataInitially = true;
        ReadFromBag();
        storeData();
      }
      storeDataInitially = false;
      egoSecVec.clear();
      ReadFromBag();
      savePlotData();
      //Publishing the msg to inform the end of this process
      std_msgs::String msg;
      msg.data = "true";
      finishPub.publish(msg);
    }

    void LoadAndSaveLaneMapWithLatLong(){
      lanelet::projection::UtmProjector projector(lanelet::Origin({-33.887362384900044, 151.19899900328662}));
      auto tempMap = lanelet::load(lanelet_file, projector);
      lanelet::write("/model/testMap.osm", *tempMap, projector);

    }

    void loadData(){
      std::ifstream i1(centerline_json_file);
      json j1;i1 >> j1; 
      for (auto& item : j1["road_center"]) {
        lanelet::Point3d p1{lanelet::utils::getId(), item["x"], item["y"], 0};
        roadCenterLine.push_back(p1);
      }
      for (auto& item : j1["ego_center"]) {
        lanelet::Point3d p1{lanelet::utils::getId(), item["x"], item["y"], 0};
        egoCenterLine.push_back(p1);
      }

      ROS_INFO_STREAM("Road LineString size: "<<roadCenterLine.size());
      ROS_INFO_STREAM("Ego LineString size: "<<egoCenterLine.size());

      for(auto& p: roadCenterLine){
        if(p.x() > 422)
          break;
        //ROS_INFO_STREAM(p.x()<<" "<<p.y());
      }
    }
    
    void storeData(){
      //Saving the centerline of the road to a json file
      std::vector<json> centerline;
      for(size_t i =0; i<roadCenterLine.size(); i++){
        auto point = roadCenterLine[i];
        json jData;jData["x"] = point.x();jData["y"] = point.y();
        centerline.push_back(jData);    
      }
      json j1(centerline);
      std::vector<json> egoline;
      for(size_t i =0; i<egoCenterLine.size(); i++){
        auto point = egoCenterLine[i];
        json jData;jData["x"] = point.x();jData["y"] = point.y();
        egoline.push_back(jData);    
      }
      json j2(egoline);
      json centerLineJson = {
        {"road_center", j1},
        {"ego_center", j2}
      }; 
      std::ofstream o1(centerline_json_file);
      o1 << std::setw(4) << centerLineJson << std::endl;
      ROS_INFO_STREAM("Road LineString size: "<<roadCenterLine.size());
      ROS_INFO_STREAM("Ego LineString size: "<<egoCenterLine.size());
      centerline.clear();
      egoline.clear();
      
    }
         
    void MessagePublisher(ros::Publisher &publisher, const rosbag::MessageInstance &message) {
      if(storeDataInitially){
        if(message.getTopic() == odometry_topic){
          nav_msgs::Odometry::ConstPtr odomPtr = message.instantiate<nav_msgs::Odometry>();
          if(odomPtr != nullptr){
            buildCenterLine(odomPtr); 
            buildEgoPathInLaneletCenterline(odomPtr);
            ROS_INFO_STREAM("Storing... Time in sec: "<<odomPtr->header.stamp.sec);
          }
        }//odometry topic if close
      }else{
        current_message_number++;
        int current_percentage = 100.0 * ((float) current_message_number / (float) total_message_count);
        if (current_percentage != previous_percentage) {
          previous_percentage = current_percentage;
          if (current_percentage == 0) {
            std::cout << "Reading the bag file started" << std::endl;
          } else {
            end_time = std::chrono::steady_clock::now();
            std::chrono::duration<float, std::ratio<60>> elapsed = end_time - start_time;
            float estimated_remaining = (float) (100 - current_percentage) * elapsed.count();
            std::cout << current_percentage << "%, est. remaining " << estimated_remaining << " minutes" << std::endl;
          }
          start_time = std::chrono::steady_clock::now();
        }
        //Publish all of the bag topics
        publisher.publish(message);
        laneChangeDetection(message);
      }
    }//MessagePublisher method closes

    void buildCenterLine(nav_msgs::Odometry::ConstPtr odomPtr){
      auto egoPoint2d = lanelet::BasicPoint2d(odomPtr->pose.pose.position.x, odomPtr->pose.pose.position.y);
      auto tuple = findTheActualLanelet(map, egoPoint2d);
      lanelet::BasicPoint2d startingPoint;
      if(std::get<0>(tuple)){
        startingPoint = lanelet::geometry::project(lanelet::utils::to2D(std::get<1>(tuple).centerline()), egoPoint2d); 
      }else{
        //Just in case map is slightly wrong, we cannot find the lanelet
        //which the egopoint resides
        auto startingPointPair = findTheClosestLaneletForObject(map, egoPoint2d);
        startingPoint = startingPointPair.second;
      }
      auto centerLineTuple = findTheCentralLinePoint(map, startingPoint);
      if(std::get<0>(centerLineTuple)){
        //auto roadCenter = std::get<1>(centerLineTuple).first; 
        for(auto& cp: std::get<1>(centerLineTuple).second){
          bool found = false;
          for(auto& rp: roadCenterLine){
            if(cp == rp)
              found = true;
          }  
          if(!found)
            roadCenterLine.push_back(cp);
        }
        //lanelet::Point3d p1{lanelet::utils::getId(), roadCenter.x(), roadCenter.y(), 0};
        //roadCenterLine.push_back(p1);
      }
      //ROS_INFO_STREAM("Road center linestring size: "<<roadCenterLine.size());
    }

    void buildEgoPathInLaneletCenterline(nav_msgs::Odometry::ConstPtr odomPtr)
    {
      if(std::find(egoSecVec.begin(), egoSecVec.end(), odomPtr->header.stamp.sec) != egoSecVec.end()){
      }else{
        auto egoPoint2d = lanelet::BasicPoint2d(odomPtr->pose.pose.position.x, odomPtr->pose.pose.position.y);
        lanelet::Lanelet lanelet;
        auto tuple = findTheActualLanelet(map, egoPoint2d);
        if(std::get<0>(tuple)){
          lanelet = std::get<1>(tuple);
        }else{
          //Just in case map is slightly wrong, we cannot find the lanelet
          //which the egopoint resides
          auto pair = findTheClosestLaneletEgo(map, egoPoint2d);
          lanelet = pair.second;
        }
        
        for(auto& p: lanelet.centerline()){
          lanelet::Point3d point3d(lanelet::utils::getId(), {p.x(), p.y(), 0});
          bool found = false;
          for(auto& rp: egoCenterLine){
            if(point3d == rp)
              found = true;
          }  
          if(!found)
            egoCenterLine.push_back(point3d);
        }
        
        egoSecVec.push_back(odomPtr->header.stamp.sec);
      }
    }
    
    void laneChangeDetection(const rosbag::MessageInstance &msg){
      if(msg.getTopic() == odometry_topic){
        nav_msgs::Odometry::ConstPtr odomPtr = msg.instantiate<nav_msgs::Odometry>();
        //if(std::find(egoSecVec.begin(), egoSecVec.end(), odomPtr->header.stamp.sec) != egoSecVec.end()){
        //}else{
          constructFrenetFrameEgo(odomPtr);
          egoSecVec.push_back(odomPtr->header.stamp.sec);
        //}
      }//Odometry topic if closes

      if(msg.getTopic() == objects_topic){
        ibeo_object_msg::IbeoObject::ConstPtr objPtr = msg.instantiate<ibeo_object_msg::IbeoObject>();
        bool proceed = checkSecObjExist(objPtr->object_id, objPtr->header.stamp.sec, objPtr->header.stamp.nsec, objSecVec);
        if(std::find(objectClassVec.begin(), objectClassVec.end(), objPtr->object_class) != objectClassVec.end() && proceed){
          constructFrenetFrameOtherCars(objPtr);
          scenarioDetection(objPtr);
          //objSecVec.push_back(std::make_pair(objPtr->object_id, objPtr->header.stamp.sec));
        }
      }
    }

    void scenarioDetection(ibeo_object_msg::IbeoObject::ConstPtr objPtr){
      auto pointPair = baselinkToOdom(objPtr, transformer_);
      if(pointPair.first &&(objPtr->pose.pose.position.x >= 0 && objPtr->pose.pose.position.x < 50)){
        auto point = pointPair.second;  
        double posX = point.x();double posY = point.y(); 
        auto point3d = lanelet::Point3d{lanelet::utils::getId(), posX, posY, 0};
        auto egoCenter = lanelet::geometry::project(egoCenterLine, point3d);
        auto egoCenterls = getTheRoadLineString(egoCenterLine, lanelet::Point3d{lanelet::utils::getId(), egoCenter.x(), egoCenter.y(), 0});
        auto laneLaneletPair = getTheLaneNo(map, point);
        if(egoCenterls.size() > 0 && currentEgoSpeed > 0.5){
          float d = std::abs(lanelet::geometry::signedDistance(lanelet::utils::to2D(egoCenterls), point));
          //if(standalone_run && objPtr->object_id == 140)
          //  ROS_INFO_STREAM("Car_id: "<<objPtr->object_id<<" d: "<<d<<" lane no: "<<laneLaneletPair.first);
          if(std::find(cutInScenarioNoDetect.begin(), cutInScenarioNoDetect.end(), objPtr->object_id) != cutInScenarioNoDetect.end() && laneLaneletPair.first != 0 && currentEgoLaneNo == laneLaneletPair.first){
            if(std::find(cutInScenarioCar.begin(), cutInScenarioCar.end(), objPtr->object_id) != cutInScenarioCar.end()){

            }else{
              ROS_INFO_STREAM("Cutin Scenario detected!!! car: "<<objPtr->object_id<<" "<<d<<" lane no: "<<laneLaneletPair.first<<" ego lane no: "<<currentEgoLaneNo);
              json jData;
              jData["scenario_start"] = objPtr->header.stamp.sec-12; //8 before
              jData["scenario_end"] = objPtr->header.stamp.sec+8; //4 before
              jData["cutin_start"] = objPtr->header.stamp.sec-5;
              jData["cutin_end"] = objPtr->header.stamp.sec+1;
              jData["cutin_car"] = objPtr->object_id;
              cutInScenarios.push_back(jData);
              cutInScenarioCar.push_back(objPtr->object_id);
            }
          }

          //Cut-out scenario detection
          //If the d >2 for the car that was on the same line as ego, then we
          //can identify it as the cut-out scenario
          else if(std::find(cutOutScenarioNoDetect.begin(), cutOutScenarioNoDetect.end(), objPtr->object_id) != cutOutScenarioNoDetect.end() && laneLaneletPair.first != 0 && currentEgoLaneNo != laneLaneletPair.first){
            if(std::find(cutOutScenarioCar.begin(), cutOutScenarioCar.end(), objPtr->object_id) != cutOutScenarioCar.end()){

            }else{
              ROS_INFO_STREAM("Cut-out Scenario detected!!! car: "<<objPtr->object_id<<" "<<d<<" lane no: "<<laneLaneletPair.first<<" ego lane no: "<<currentEgoLaneNo);
              json jData;
              jData["scenario_start"] = objPtr->header.stamp.sec-12; //8 before
              jData["scenario_end"] = objPtr->header.stamp.sec+8; //4 before
              jData["cutout_start"] = objPtr->header.stamp.sec-5;
              jData["cutout_end"] = objPtr->header.stamp.sec+1;
              jData["cutout_car"] = objPtr->object_id;
              cutOutScenarios.push_back(jData);
              cutOutScenarioCar.push_back(objPtr->object_id);
            }
          }
          
          //Detect vehicle that are on the same line as ego vehicle for cut-in 
          if(d > 1.5 && currentEgoLaneNo != laneLaneletPair.first && currentEgoLaneNo != 0 && laneLaneletPair.first != 0){
            if(std::find(cutInScenarioNoDetect.begin(), cutInScenarioNoDetect.end(), objPtr->object_id) != cutInScenarioNoDetect.end()){
              
            }else{
              cutInScenarioNoDetect.push_back(objPtr->object_id);
            }
          }
          
          // For cut-out scenario, we need to identify the car infront of the
          // vehicle on the same lane
          else if(d < 0.5 && currentEgoLaneNo == laneLaneletPair.first && currentEgoLaneNo != 0 && laneLaneletPair.first != 0){
            if(std::find(cutOutScenarioNoDetect.begin(), cutOutScenarioNoDetect.end(), objPtr->object_id) != cutOutScenarioNoDetect.end()){
              
            }else{
              //Only consider the object that are not in cut-in scenario
             // if(std::find(cutInScenarioNoDetect.begin(), cutInScenarioNoDetect.end(), objPtr->object_id) != cutInScenarioNoDetect.end()){
              //}else{
                
                cutOutScenarioNoDetect.push_back(objPtr->object_id);
                //Consider lane following scenario as well
                /*json jData;
                jData["scenario_start"] = objPtr->header.stamp.sec;
                jData["scenario_end"] = objPtr->header.stamp.sec+10;
                jData["laneFollowing_car"] = objPtr->object_id;
                laneFollowingScenarios.push_back(jData);
                laneFollowingCar.push_back(objPtr->object_id);*/
              //}
            }
          }
        } 
      }
    }
        
    void constructFrenetFrameEgo(nav_msgs::Odometry::ConstPtr odomPtr){
      json jData; json odomJdata; json otherJdata;
      double egoPosX = odomPtr->pose.pose.position.x;double egoPosY = odomPtr->pose.pose.position.y; 
      auto egoPoint = lanelet::BasicPoint2d(egoPosX, egoPosY);
      auto egoPoint3d = lanelet::Point3d{lanelet::utils::getId(), egoPosX, egoPosY, 0};
      auto roadCenter = lanelet::geometry::project(roadCenterLine, egoPoint3d);
      auto roadCenterls = getTheRoadLineString(roadCenterLine, lanelet::Point3d{lanelet::utils::getId(), roadCenter.x(), roadCenter.y(), 0});
      auto laneLaneletPair = getTheLaneNo(map, egoPoint, "ego");
      currentEgoLaneNo = laneLaneletPair.first;
      currentEgoSpeed = odomPtr->twist.twist.linear.x;
      if(laneLaneletPair.first != 0){ 
        //ROS_INFO_STREAM("Ego lane no: "<<laneLaneletPair.first);
        frenetS = getFrenetSEgo(frenetJsonEgo, roadCenterLine, lanelet::BasicPoint2d(roadCenter.x(), roadCenter.y()));
        float d = lanelet::geometry::signedDistance(lanelet::utils::to2D(roadCenterls), egoPoint);
        
        if(d >= 0)
          positiveDir++;
        else
          negativeDir++;

        if(egoDataCount > 10 && positiveDir > negativeDir && d < 0)
            d *= -1;
        //ROS_INFO_STREAM("roadCenterLs: "<<roadCenterls.front().x()<<" "<<roadCenterls.front().y()<<" "<<roadCenterls.back().x()<<" "<<roadCenterls.back().y()<<" egoDataCount: "<<egoDataCount<<" positiveDir: "<<positiveDir<<" negativeDir: "<<negativeDir<<" , egoPoint: "<<egoPoint.x()<<" "<<egoPoint.y()<<" d:"<<d);
        jData["s"] = frenetS;
        jData["d"] = d;
        jData["sec"] = odomPtr->header.stamp.sec;
        odomJdata["x"] = egoPosX;
        odomJdata["y"] = egoPosY;
        odomJdata["sec"] = odomPtr->header.stamp.sec;
        otherJdata["long_speed"] = odomPtr->twist.twist.linear.x;
        otherJdata["lane_no"] = laneLaneletPair.first;
        frenetJsonVecEgo(frenetJsonEgo,std::make_pair(jData,odomJdata),lanelet::BasicPoint2d(roadCenter.x(), roadCenter.y()), frenetS, otherJdata);
        egoDataCount++;
      }
    }

    void constructFrenetFrameOtherCars(ibeo_object_msg::IbeoObject::ConstPtr objPtr){
      json jData; json odomJdata; json otherJdata;
      int car = objPtr->object_id;
      //ROS_INFO_STREAM("car: "<<car);
      auto pointPair = baselinkToOdom(objPtr, transformer_);
      if(pointPair.first){
        auto point = pointPair.second;  
        double posX = point.x();double posY = point.y(); 
        //ROS_INFO_STREAM("posX: "<<posX<<" posY:"<<posY);
        auto point3d = lanelet::Point3d{lanelet::utils::getId(), posX, posY, 0};
        auto roadCenter = lanelet::geometry::project(roadCenterLine, point3d);
        auto roadCenterls = getTheRoadLineString(roadCenterLine, lanelet::Point3d{lanelet::utils::getId(), roadCenter.x(), roadCenter.y(), 0});
        auto laneLaneletPair = getTheLaneNo(map, point);
        if(laneLaneletPair.first != 0){ 
          
          //For constructing road ceneter as reference line in Frenet frame
          auto frenetSO = getFrenetS(frenetJson, car, roadCenterLine, point3d, lanelet::BasicPoint2d(roadCenter.x(), roadCenter.y()));
          float d = lanelet::geometry::signedDistance(lanelet::utils::to2D(roadCenterls), point); 
          if(egoDataCount > 10 && positiveDir > negativeDir && d < 0)
            d *= -1;
          
          //For constructing ego path as reference line in Frenet frame
          auto egoCenter = lanelet::geometry::project(egoCenterLine, point3d);
          /*auto egoCenterls = getTheRoadLineString(egoCenterLine, lanelet::Point3d{lanelet::utils::getId(), egoCenter.x(), egoCenter.y(), 0});
          auto frenetSOEgo = getFrenetSEgoRef(frenetJsonOtherEgoRef, car, egoCenterLine, point3d, lanelet::BasicPoint2d(egoCenter.x(), egoCenter.y()));
          float d_ego_ref = lanelet::geometry::signedDistance(lanelet::utils::to2D(egoCenterls), point); 
          if(egoDataCount > 10 && positiveDir > negativeDir && d_ego_ref < 0)
            d_ego_ref *= -1;*/


          jData["s"] = frenetSO;
          jData["d"] = d;
          //jData["s_ego_ref"] = frenetSOEgo;
          //jData["d_ego_ref"] = d_ego_ref;
          jData["sec"] = objPtr->header.stamp.sec;
          odomJdata["x"] = posX;
          odomJdata["y"] = posY;
          odomJdata["sec"] = objPtr->header.stamp.sec;
          otherJdata["long_speed"] = objPtr->twist.twist.linear.x;
          otherJdata["lane_no"] = laneLaneletPair.first;
          //ROS_INFO_STREAM("car: "<<car<<" Lane: "<<laneLaneletPair.first);
          frenetJsonVec(frenetJson,std::make_tuple(car, jData,odomJdata),lanelet::BasicPoint2d(roadCenter.x(), roadCenter.y()), frenetSO, otherJdata, lanelet::BasicPoint2d(egoCenter.x(), egoCenter.y()));
        } 
      }
    }
    
    void savePlotData(){
      ROS_INFO_STREAM("Car frenet frame plot data size: "<<frenetJson.size()); 
      std::vector<json> storeJson; 
      for(auto& pair: frenetJson){
        json mainData = pair.second;
        storeJson.push_back(mainData);
      } 
      json dataJ1(storeJson);
      std::ofstream o1(cars_frenet_json_file);
      o1 << std::setw(4) << dataJ1 << std::endl;
      
      ROS_INFO_STREAM("Ego frenet frame plot data size: "<<frenetJsonEgo.size()); 
      storeJson.clear();
      for(auto& jData: frenetJsonEgo){
        json mainData = jData;
        storeJson.push_back(mainData);
      } 
      json dataJ2(storeJson);
      std::ofstream o2(ego_frenet_json_file);
      o2 << std::setw(4) << dataJ2 << std::endl;

      //Storing the scenario data
      std::vector<json> cutinScenarioJdata;
      for(size_t i =0; i<cutInScenarios.size(); i++){
        auto jData = cutInScenarios[i]; 
        cutinScenarioJdata.push_back(jData);    
      }
      json j3(cutinScenarioJdata);
      std::vector<json> cutoutScenarioJdata;
      for(size_t i =0; i<cutOutScenarios.size(); i++){
        auto jData = cutOutScenarios[i]; 
        cutoutScenarioJdata.push_back(jData);    
      }
      json j4(cutoutScenarioJdata);
      
      //Process the lanefollowing scenario sucha way that lane followinf car is
      //not in both cut-in and cut-out scenarios
      /*std::vector<json> laneFollowingS;
      for(auto& s: laneFollowingScenarios){
        auto car = s["laneFollowing_car"];
        if(std::find(cutInScenarioCar.begin(), cutInScenarioCar.end(), car) != cutInScenarioCar.end() || std::find(cutOutScenarioCar.begin(), cutOutScenarioCar.end(), car) != cutOutScenarioCar.end()){
          continue; 
        }else{
          laneFollowingS.push_back(s);
        }
      }
      json j5(laneFollowingS);*/

      json scenarioJson = {
        {"cut-in scenario", j3},
        {"cut-out scenario", j4},
        //{"lane-following scenario", j5},
        {"file", bag_file}
      };
      std::ofstream o3(scenario_json_file);
      o3 << std::setw(4) << scenarioJson << std::endl;

    }
};


int main(int argc, char **argv) {

  
  ros::init(argc, argv, "Extraction");
  Extraction extract;
  extract.init_playback();
  if(extract.standalone_run){
    ROS_INFO_STREAM("Resume: "<<extract.resume);
    if(extract.resume){
      ROS_INFO_STREAM("Data is loading from JSON files...");
      extract.loadData();
    }else{
      ROS_INFO_STREAM("Data is loading from Bag file...");
      extract.storeDataInitially = true;  
      extract.ReadFromBag();
      extract.storeData();
    }
    extract.storeDataInitially = false;
    extract.egoSecVec.clear();
    extract.ReadFromBag();
    extract.savePlotData();
    //Oublishing the msg to inform the end of this process
    std_msgs::String msg;
    msg.data = "true";
    extract.finishPub.publish(msg);
  }
  ros::spin();

  return 0;
}

