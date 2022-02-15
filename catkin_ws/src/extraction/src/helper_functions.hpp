#ifndef APPLICATIONS_HELPER_FUNCTIONS_HPP
#define APPLICATIONS_HELPER_FUNCTIONS_HPP

using json = nlohmann::json;

json constructJsonData(nav_msgs::Odometry::ConstPtr dataPtr, std::shared_ptr<tf2_ros::Buffer> transformer_){
  json jData;
  /*tf::Quaternion q(
    dataPtr->pose.pose.orientation.x,
    dataPtr->pose.pose.orientation.y,
    dataPtr->pose.pose.orientation.z,
    dataPtr->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  jData["linear_x"] = dataPtr->twist.twist.linear.x;
  jData["linear_y"] = dataPtr->twist.twist.linear.y;
  jData["linear_z"] = dataPtr->twist.twist.linear.z;*/
  jData["position_x"] = dataPtr->pose.pose.position.x;
  jData["position_y"] = dataPtr->pose.pose.position.y;
  /*jData["position_z"] = dataPtr->pose.pose.position.z;
  jData["roll"] = roll;
  jData["pitch"] = pitch;
  jData["yaw"] = yaw;*/
  jData["sec"] = dataPtr->header.stamp.sec;
  jData["nsec"] = dataPtr->header.stamp.nsec;

  return jData;
}

json constructJsonData(ibeo_object_msg::IbeoObject::ConstPtr dataPtr, std::shared_ptr<tf2_ros::Buffer>& transformer_){
  json jData;
  try {
    /*tf::Quaternion q(
      dataPtr->pose.pose.orientation.x,
      dataPtr->pose.pose.orientation.y,
      dataPtr->pose.pose.orientation.z,
      dataPtr->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);*/
    geometry_msgs::PointStamped baseLinkPoint;
    baseLinkPoint.point.x = dataPtr->pose.pose.position.x;baseLinkPoint.point.y = dataPtr->pose.pose.position.y;baseLinkPoint.point.z = 0;
    baseLinkPoint.header.frame_id = "base_link";baseLinkPoint.header.stamp= dataPtr->header.stamp;
    geometry_msgs::PointStamped odomPoint;
    transformer_->setUsingDedicatedThread(true);
    transformer_->transform(baseLinkPoint, odomPoint, "odom");
    jData["object_id"] = dataPtr->object_id;
    /*jData["linear_x"] = dataPtr->twist.twist.linear.x;
    jData["linear_y"] = dataPtr->twist.twist.linear.y;
    jData["linear_z"] = dataPtr->twist.twist.linear.z;*/
    jData["position_x"] = odomPoint.point.x;
    jData["position_y"] = odomPoint.point.y;
    jData["pos_baselink_x"] = dataPtr->pose.pose.position.x;
    jData["pos_baselink_y"] = dataPtr->pose.pose.position.y;
    /*jData["pos_baselink_z"] = dataPtr->pose.pose.position.z;
    jData["roll"] = roll;
    jData["pitch"] = pitch;
    jData["yaw"] = yaw;*/
    jData["sec"] = dataPtr->header.stamp.sec;
    jData["nsec"] = dataPtr->header.stamp.nsec;
  }catch (const std::exception &e) {
    ROS_ERROR_STREAM(e.what());
  }

  return jData;
}

std::pair<bool,lanelet::BasicPoint2d> baselinkToOdom(ibeo_object_msg::IbeoObject::ConstPtr dataPtr, std::shared_ptr<tf2_ros::Buffer>& transformer_){
  lanelet::BasicPoint2d point;
  std::pair<bool, lanelet::BasicPoint2d> _return = std::make_pair(false, point);  
  try {
    geometry_msgs::PointStamped baseLinkPoint;
    baseLinkPoint.point.x = dataPtr->pose.pose.position.x;baseLinkPoint.point.y = dataPtr->pose.pose.position.y;baseLinkPoint.point.z = 0;
    baseLinkPoint.header.frame_id = "base_link";baseLinkPoint.header.stamp= dataPtr->header.stamp;
    geometry_msgs::PointStamped odomPoint;
    transformer_->setUsingDedicatedThread(true);
    transformer_->transform(baseLinkPoint, odomPoint, "odom");
    lanelet::BasicPoint2d point(odomPoint.point.x, odomPoint.point.y);
    _return = std::make_pair(true, point);
  }catch (const std::exception &e) {
    ROS_ERROR_STREAM(e.what());
  }

  return _return;
}

bool checkSecExist(std::vector<uint32_t> timeStamp, uint32_t sec){
  bool _return = false;
  if(std::find(timeStamp.begin(), timeStamp.end(), sec) != timeStamp.end())  {
    _return = true;
  }
  
  return _return;
}

int findIndex(std::vector<uint32_t> v, uint32_t sec){
  int _return = -1;  
  auto it = std::find(v.begin(), v.end(), sec);
 
  // If element was found
  if (it != v.end())
    _return = it - v.begin();

  return _return;
}

std::tuple<bool, lanelet::Lanelet, std::vector<lanelet::Lanelet>> findTheActualLanelet(lanelet::LaneletMapPtr map, lanelet::BasicPoint2d egoPoint){
  std::vector<std::pair<double, lanelet::Lanelet>> nearLanelets = lanelet::geometry::findNearest(map->laneletLayer, egoPoint, 10);
  bool llFound = false;
  size_t selected = 0;
  std::vector<lanelet::Lanelet> lanelets;
  for(size_t i=0; i<nearLanelets.size(); i++){
    auto ll = nearLanelets[i];
    if(lanelet::geometry::inside(ll.second, egoPoint)){
      llFound = true;
      selected = i;
      break;
    }
  }

  for(size_t i=0; i<nearLanelets.size(); i++){
    if(llFound && i == selected)
      continue;
    else{
      lanelets.push_back(nearLanelets[i].second);
    }
  }
  
  if(llFound)
    return std::make_tuple(true, nearLanelets[selected].second, lanelets);
  else{
    lanelet::Lanelet tempLL;
    return std::make_tuple(false, tempLL, lanelets);
  }
}

std::pair<bool,lanelet::Lanelet> findTheLeftLanelet(lanelet::Lanelet checkLanelet, std::vector<lanelet::Lanelet> lanelets){
  lanelet::Lanelet lanelet;
  std::pair<bool, lanelet::Lanelet> _return = std::make_pair(false, lanelet);
  for(size_t i=0; i<lanelets.size(); i++){
    if(lanelet::geometry::leftOf(lanelets[i], checkLanelet)){
      _return = std::make_pair(true, lanelets[i]);
      break;
    }
  }
  
  return _return;
}

std::pair<bool,lanelet::Lanelet> findTheRightLanelet(lanelet::Lanelet checkLanelet, std::vector<lanelet::Lanelet> lanelets){
  lanelet::Lanelet lanelet;
  std::pair<bool, lanelet::Lanelet> _return = std::make_pair(false, lanelet);
  for(size_t i=0; i<lanelets.size(); i++){
    if(lanelet::geometry::rightOf(lanelets[i], checkLanelet)){
      _return = std::make_pair(true, lanelets[i]);
      break;
    }
  }
  
  return _return;
}


std::pair<int,int> findTheNumberOfLanesAndCarLane(lanelet::LaneletMapPtr map, lanelet::BasicPoint2d startingPoint){
  std::pair<int, int> _return = std::make_pair(0, -1);// number of lanes, position of the car
  auto nearLanelets = findTheActualLanelet(map, startingPoint);
  if(std::get<0>(nearLanelets) && lanelet::geometry::length3d(std::get<1>(nearLanelets)) > 0){
    auto firstLanelet = std::get<1>(nearLanelets);
    auto currentLanelet = firstLanelet;
    auto lanelets = std::get<2>(nearLanelets);
    std::vector<std::pair<int, lanelet::Lanelet>> selectedLanelets;
    int count = 0;
    //Find all the left lanelets
    while(true){
      auto result = findTheLeftLanelet(currentLanelet, lanelets);
      if(result.first){
        selectedLanelets.push_back(std::make_pair(count, result.second));
        currentLanelet = result.second;
        lanelet::ConstLineString3d centerline = currentLanelet.centerline();
        int midSegNo = centerline.numSegments()/2;
        auto midSeg = centerline.segment(midSegNo);
        auto point1 = midSeg.first;auto point2 = midSeg.second;
        auto midX = (point1.x()+point2.x())/2; auto midY = (point1.y()+point2.y())/2;
        auto point = lanelet::BasicPoint2d(midX, midY);
        auto otherLanelets = findTheActualLanelet(map, point);
        lanelets = std::get<2>(otherLanelets);
        count++;
      }else
        break;
    }
    
    std::sort(selectedLanelets.begin(), selectedLanelets.end(), [](const std::pair<int, lanelet::Lanelet>& a, std::pair<int, lanelet::Lanelet>& b) {return a.first < b.first;});
    std::vector<lanelet::Lanelet> allLanelets;
    for(auto& pair: selectedLanelets)
      allLanelets.push_back(pair.second);
    
    int carLane = selectedLanelets.size();
    allLanelets.push_back(firstLanelet);

    //Find all the right lanelets
    currentLanelet = firstLanelet;
    lanelets = std::get<2>(nearLanelets);
    count = 0;
    selectedLanelets.clear();
    while(true){
      auto result = findTheRightLanelet(currentLanelet, lanelets);
      if(result.first){
        allLanelets.push_back(result.second);
        currentLanelet = result.second;
        lanelet::ConstLineString3d centerline = currentLanelet.centerline();  
        int midSegNo = centerline.numSegments()/2;
        auto midSeg = centerline.segment(midSegNo);
        auto point1 = midSeg.first;auto point2 = midSeg.second;
        auto midX = (point1.x()+point2.x())/2; auto midY = (point1.y()+point2.y())/2;
        auto point = lanelet::BasicPoint2d(midX, midY);
        auto otherLanelets = findTheActualLanelet(map, point);
        lanelets = std::get<2>(otherLanelets);
        count++;
      }else
        break;
    }

    _return = std::make_pair(allLanelets.size(), carLane);
  
  }//Checking lanelet for the egopoint if closes      
  else{
    //ROS_INFO_STREAM("No lanelet found: length of the lanelet is zero or simply couldn't find the lanelet that actually has the point");
  }

  return _return;
}

void groupObjDataBySec(json jData, std::vector<std::pair<uint32_t, std::vector<json>>>& objectsPerSec){
  bool found = false;
  size_t index = 0;
  for(size_t i=0; i<objectsPerSec.size(); i++){
    auto pair = objectsPerSec[i];
    if(pair.first == jData["sec"].get<uint32_t>()){
      found = true;
      index = i;
    }
  }
  if(found){
    auto pair = objectsPerSec[index]; 
    auto vec = pair.second;
    vec.push_back(jData);
    objectsPerSec[index] = std::make_pair(pair.first, vec);
  }else{
    std::vector<json> tempVec; tempVec.push_back(jData);
    objectsPerSec.push_back(std::make_pair(jData["sec"].get<uint32_t>(), tempVec));
  }
}

void groupObjDataByCar(json jData, std::vector<std::pair<int, std::vector<json>>>& groupByCar){
  bool found = false;
  size_t index = 0;
  for(size_t i=0; i<groupByCar.size(); i++){
    auto pair = groupByCar[i];
    if(pair.first == jData["object_id"].get<int>()){
      found = true;
      index = i;
    }
  }
  if(found){
    auto pair = groupByCar[index]; 
    auto vec = pair.second;
    bool proceed = true;
    for(size_t i=0; i<vec.size(); i++){
      auto j = vec[i];
      if(j["sec"] == jData["sec"]){
        proceed = false;
      }
    }
    if(proceed){
      vec.push_back(jData);
      groupByCar[index] = std::make_pair(pair.first, vec);
    }
  }else{
    std::vector<json> tempVec; tempVec.push_back(jData);
    groupByCar.push_back(std::make_pair(jData["object_id"].get<int>(), tempVec));
  }
}

void groupObjects(std::vector<json> objectsPos, std::vector<std::pair<uint32_t, std::vector<json>>>& groupBySec, std::vector<std::pair<int, std::vector<json>>>& groupByCar){
  for(auto& item: objectsPos){
    groupObjDataBySec(item, groupBySec);
    groupObjDataByCar(item, groupByCar);
  }
}

std::vector<json> getAllTheCarsAtSec(std::vector<std::pair<uint32_t, std::vector<json>>> groupBySec, uint32_t sec){
 
  std::vector<json> carsAtSec;
  for(auto& pair: groupBySec){
    if(pair.first == sec){
      carsAtSec = pair.second;
    }
  }
  
  return carsAtSec;
}

std::vector<json> getAllTheDataAtCar(std::vector<std::pair<int, std::vector<json>>> groupByCar, int car){
 
  std::vector<json> dataAtCar;
  for(auto& pair: groupByCar){
    if(pair.first == car){
      dataAtCar = pair.second;
    }
  }
  
  return dataAtCar;
}

std::vector<json> getEgoDataTillProj(std::vector<uint32_t> projectedSec, std::vector<json> odomPos, std::vector<uint32_t> odomTimeStamp){

  std::vector<json> returnVec;
  for(auto& sec: projectedSec){
      auto index = findIndex(odomTimeStamp, sec);
      if(index != -1)
        returnVec.push_back(odomPos[index]);
  }

  return returnVec;
}

std::vector<double> getObjOdomPos(double x1, double y1){
  std::vector<double> returnVec;
  returnVec.push_back(x1);
  returnVec.push_back(y1);
  
  return returnVec;  
}

std::pair<bool,json> getCarDataAtSec(std::vector<json> dataAtCar, uint32_t sec){

  json _returnJ;
  bool found = false;
  for(auto& j: dataAtCar){
    if(sec == j["sec"]){
      found = true;
      _returnJ = j;
      break;
    }
  }
  
  return std::make_pair(found, _returnJ);
}

lanelet::BasicPoint2d findTheCorrespondingEgoPoints(std::vector<json> odomPos, lanelet::BasicPoint2d objectPoint){
  lanelet::BasicPoint2d egoPoint;
  auto x1 = objectPoint.x(); auto y1 = objectPoint.y(); 
  double smallD = 1000.;
  size_t index = 0;
  bool found = false;
  for(size_t i=0;i<odomPos.size();i++){
    auto jData = odomPos[i];
    auto x2 = jData["position_x"].get<double>(); 
    auto y2 = jData["position_x"].get<double>();
    float d = std::sqrt(std::pow((x2-x1),2)+std::pow((y2-y1),2));
    if(d<smallD){
      smallD = d;
      index = i;
      found = true;
    }
  }
  
  if(found){
    auto data = odomPos[index];    
    double egoPosX = data["position_x"];double egoPosY = data["position_y"];
    egoPoint = lanelet::BasicPoint2d(egoPosX, egoPosY);
  }

  return egoPoint;
}

std::pair<bool, lanelet::BasicPoint2d> findTheClosestLaneletForObject(lanelet::LaneletMapPtr map, lanelet::BasicPoint2d startingPoint){
  std::vector<std::pair<double, lanelet::Lanelet>> nearLanelets = lanelet::geometry::findNearest(map->laneletLayer, startingPoint, 10);
  std::vector<lanelet::Lanelet> lanelets;
  lanelet::BasicPoint2d point2d;
  double smallD = 1000.; size_t selectedIndex = 0; bool found = false;
  for(size_t i=0; i<nearLanelets.size(); i++){
    if(nearLanelets[i].first < smallD && nearLanelets[i].first < 10){
      smallD = nearLanelets[i].first;
      selectedIndex = i;
      found = true;
    }
    lanelets.push_back(nearLanelets[i].second);
  }
  if(found){
    auto lanelet = lanelets[selectedIndex];
    auto centerLine = lanelet.centerline();
    auto pProj = lanelet::geometry::project(centerLine, lanelet::Point3d{lanelet::utils::getId(), startingPoint.x(), startingPoint.y()});
    point2d = lanelet::BasicPoint2d(pProj.x(), pProj.y());
  }

  return std::make_pair(found, point2d);
}

std::pair<int,std::vector<std::pair<int,lanelet::Lanelet>>> getTheLaneNo(lanelet::LaneletMapPtr map, lanelet::BasicPoint2d startingPoint, std::string type="other"){
  auto nearLanelets = findTheActualLanelet(map, startingPoint);
  std::vector<std::pair<int,lanelet::Lanelet>> lanelanelet;
  int laneNo = 0; 
  if(std::get<0>(nearLanelets) && lanelet::geometry::length3d(std::get<1>(nearLanelets)) > 0){
    auto firstLanelet = std::get<1>(nearLanelets);
    auto currentLanelet = firstLanelet;
    auto lanelets = std::get<2>(nearLanelets);
    std::vector<std::pair<int, lanelet::Lanelet>> selectedLanelets;
    int count = 0;

    //Find all the left lanelets
    while(true){
      auto result = findTheLeftLanelet(currentLanelet, lanelets);
      if(result.first){
        selectedLanelets.push_back(std::make_pair(count, result.second));
        currentLanelet = result.second;
        lanelet::ConstLineString3d centerline = currentLanelet.centerline();
        int midSegNo = centerline.numSegments()/2;
        auto midSeg = centerline.segment(midSegNo);
        auto point1 = midSeg.first;auto point2 = midSeg.second;
        auto midX = (point1.x()+point2.x())/2; auto midY = (point1.y()+point2.y())/2;
        auto point = lanelet::BasicPoint2d(midX, midY);
        auto otherLanelets = findTheActualLanelet(map, point);
        lanelets = std::get<2>(otherLanelets);
        count++;
      }else
        break;
    }
    
    std::sort(selectedLanelets.begin(), selectedLanelets.end(), [](const std::pair<int, lanelet::Lanelet>& a, std::pair<int, lanelet::Lanelet>& b) {return a.first < b.first;});
    std::vector<lanelet::Lanelet> allLanelets;
    for(auto& pair: selectedLanelets)
      allLanelets.push_back(pair.second);
    
    allLanelets.push_back(firstLanelet);
    
    //Find all the right lanelets
    currentLanelet = firstLanelet;
    lanelets = std::get<2>(nearLanelets);
    count = 0;
    selectedLanelets.clear();
    while(true){
      auto result = findTheRightLanelet(currentLanelet, lanelets);
      if(result.first){
        allLanelets.push_back(result.second);
        currentLanelet = result.second;
        lanelet::ConstLineString3d centerline = currentLanelet.centerline();  
        int midSegNo = centerline.numSegments()/2;
        auto midSeg = centerline.segment(midSegNo);
        auto point1 = midSeg.first;auto point2 = midSeg.second;
        auto midX = (point1.x()+point2.x())/2; auto midY = (point1.y()+point2.y())/2;
        auto point = lanelet::BasicPoint2d(midX, midY);
        auto otherLanelets = findTheActualLanelet(map, point);
        lanelets = std::get<2>(otherLanelets);
        count++;
      }else
        break;
    }
    
    count = -1; 
    for(size_t i=0; i<allLanelets.size(); i++){
      lanelanelet.push_back(std::make_pair(count, allLanelets[i]));
      count--;
    } 
    
    /*
    if(allLanelets.size()%2 == 0){
      //even then, find the two middle two lanelets and get the commly shared
      //linestring as the center point.
      int rem = allLanelets.size()/2;
      rem -=1;
      count = 1;
      for(size_t i=rem; i>=0; i--){
        lanelanelet.push_back(std::make_pair(count, allLanelets[i]));
        count++;
        if(i==0)
          break;
      }
      count = -1;
      for(size_t i=rem+1; i<allLanelets.size(); i++){
        lanelanelet.push_back(std::make_pair(count, allLanelets[i]));
        count--;
      }

    }else{
      //For odd number
      int rem = allLanelets.size()/2;
      count = 1;
      for(size_t i=rem; i>=0; i--){
        lanelanelet.push_back(std::make_pair(count, allLanelets[i]));
        count++;
        if(i==0)
          break;
      }
      count = -1;
      for(size_t i=rem+1; i<allLanelets.size(); i++){
        lanelanelet.push_back(std::make_pair(count, allLanelets[i]));
        count--;
      }
    }*/
    
    for(size_t i=0; i<lanelanelet.size();i++){
      if(lanelanelet[i].second == firstLanelet)
        laneNo = lanelanelet[i].first;
      //ROS_INFO_STREAM("Lanes: "<<lanelanelet[i].first);
    }
  }//main if closes

  return std::make_pair(laneNo,lanelanelet);
}

std::tuple<bool,std::pair<lanelet::BasicPoint2d, lanelet::LineString3d>, std::pair<lanelet::Lanelet, int>> findTheCentralLinePoint(lanelet::LaneletMapPtr map, lanelet::BasicPoint2d startingPoint){
  lanelet::BasicPoint2d basicPoint3d;
  lanelet::Lanelet sampleL;
  lanelet::LineString3d sampleLs;
  bool found = false;
  std::tuple<bool,std::pair<lanelet::BasicPoint2d, lanelet::LineString3d>, std::pair<lanelet::Lanelet, int>> _return = std::make_tuple(found, std::make_pair(basicPoint3d, sampleLs), std::make_pair(sampleL,0));
  auto nearLanelets = findTheActualLanelet(map, startingPoint);
  if(std::get<0>(nearLanelets) && lanelet::geometry::length3d(std::get<1>(nearLanelets)) > 0){
    auto firstLanelet = std::get<1>(nearLanelets);
    auto currentLanelet = firstLanelet;
    auto lanelets = std::get<2>(nearLanelets);
    std::vector<std::pair<int, lanelet::Lanelet>> selectedLanelets;
    int count = 0;

    //Find all the left lanelets
    while(true){
      auto result = findTheLeftLanelet(currentLanelet, lanelets);
      if(result.first){
        selectedLanelets.push_back(std::make_pair(count, result.second));
        currentLanelet = result.second;
        lanelet::ConstLineString3d centerline = currentLanelet.centerline();
        int midSegNo = centerline.numSegments()/2;
        auto midSeg = centerline.segment(midSegNo);
        auto point1 = midSeg.first;auto point2 = midSeg.second;
        auto midX = (point1.x()+point2.x())/2; auto midY = (point1.y()+point2.y())/2;
        auto point = lanelet::BasicPoint2d(midX, midY);
        auto otherLanelets = findTheActualLanelet(map, point);
        lanelets = std::get<2>(otherLanelets);
        count++;
      }else
        break;
    }
    
    std::sort(selectedLanelets.begin(), selectedLanelets.end(), [](const std::pair<int, lanelet::Lanelet>& a, std::pair<int, lanelet::Lanelet>& b) {return a.first < b.first;});
    std::vector<lanelet::Lanelet> allLanelets;
    for(auto& pair: selectedLanelets)
      allLanelets.push_back(pair.second);
    
    allLanelets.push_back(firstLanelet);
    
    //Find all the right lanelets
    currentLanelet = firstLanelet;
    lanelets = std::get<2>(nearLanelets);
    count = 0;
    selectedLanelets.clear();
    while(true){
      auto result = findTheRightLanelet(currentLanelet, lanelets);
      if(result.first){
        allLanelets.push_back(result.second);
        currentLanelet = result.second;
        lanelet::ConstLineString3d centerline = currentLanelet.centerline();  
        int midSegNo = centerline.numSegments()/2;
        auto midSeg = centerline.segment(midSegNo);
        auto point1 = midSeg.first;auto point2 = midSeg.second;
        auto midX = (point1.x()+point2.x())/2; auto midY = (point1.y()+point2.y())/2;
        auto point = lanelet::BasicPoint2d(midX, midY);
        auto otherLanelets = findTheActualLanelet(map, point);
        lanelets = std::get<2>(otherLanelets);
        count++;
      }else
        break;
    }

    //Find the point in the left linestring of the left most lanelet
    auto lsLeft = allLanelets[0].leftBound();

    //Find the points in the right linestring of the rightmost lanelet where we
    //can project the point
    auto lsRight = allLanelets[allLanelets.size()-1].rightBound();
   
    //Find the distance between the two points that is considered as the width
    //of the road
    lanelet::Lanelet boundaryLanelet(lanelet::utils::getId(), lsLeft, lsRight);
    if(allLanelets.size()%2 == 0){
      //even then, find the two middle two lanelets and get the commly shared
      //linestring as the center point.
      int rem = allLanelets.size()/2;
      //if(rem == 1)
      //  rem = 0; 
      rem -=1;
      auto firstLL = allLanelets[rem];
      auto centerLine = firstLL.rightBound();
      if(centerLine.size() > 0){
        auto pProj = lanelet::geometry::project(centerLine, lanelet::Point3d{lanelet::utils::getId(), startingPoint.x(), startingPoint.y()});
        _return = std::make_tuple(true, std::make_pair(lanelet::BasicPoint2d(pProj.x(), pProj.y()),centerLine), std::make_pair(boundaryLanelet, allLanelets.size()));
      }
    }else{
      //For odd number
      int rem = allLanelets.size()/2;
      auto lanelet = allLanelets[rem];
      auto centerLine = lanelet.rightBound();
      auto pProj = lanelet::geometry::project(centerLine, lanelet::Point3d{lanelet::utils::getId(), startingPoint.x(), startingPoint.y()});
      _return = std::make_tuple(true, std::make_pair(lanelet::BasicPoint2d(pProj.x(), pProj.y()),centerLine), std::make_pair(boundaryLanelet,allLanelets.size()));
    }
     
  }//Checking lanelet for the egopoint if closes      
  else{
    //ROS_INFO_STREAM("No lanelet found: length of the lanelet is zero or simply couldn't find the lanelet that actually has the point");
  }

  return _return;
}



/*std::tuple<bool,std::pair<lanelet::BasicPoint2d, lanelet::LineString3d>, std::pair<lanelet::Lanelet, int>> findTheCentralLinePoint(lanelet::LaneletMapPtr map, lanelet::BasicPoint2d startingPoint){
  lanelet::BasicPoint2d basicPoint3d;
  lanelet::Lanelet sampleL;
  lanelet::LineString3d sampleLs;
  bool found = false;
  std::tuple<bool,std::pair<lanelet::BasicPoint2d, lanelet::LineString3d>, std::pair<lanelet::Lanelet, int>> _return = std::make_tuple(found, std::make_pair(basicPoint3d, sampleLs), std::make_pair(sampleL,0));
  auto nearLanelets = findTheActualLanelet(map, startingPoint);
  if(std::get<0>(nearLanelets) && lanelet::geometry::length3d(std::get<1>(nearLanelets)) > 0){
    auto firstLanelet = std::get<1>(nearLanelets);
    auto currentLanelet = firstLanelet;
    auto lanelets = std::get<2>(nearLanelets);
    std::vector<std::pair<int, lanelet::Lanelet>> selectedLanelets;
    int count = 0;

    //Find all the left lanelets
    while(true){
      auto result = findTheLeftLanelet(currentLanelet, lanelets);
      if(result.first){
        selectedLanelets.push_back(std::make_pair(count, result.second));
        currentLanelet = result.second;
        lanelet::ConstLineString3d centerline = currentLanelet.centerline();
        int midSegNo = centerline.numSegments()/2;
        auto midSeg = centerline.segment(midSegNo);
        auto point1 = midSeg.first;auto point2 = midSeg.second;
        auto midX = (point1.x()+point2.x())/2; auto midY = (point1.y()+point2.y())/2;
        auto point = lanelet::BasicPoint2d(midX, midY);
        auto otherLanelets = findTheActualLanelet(map, point);
        lanelets = std::get<2>(otherLanelets);
        count++;
      }else
        break;
    }
    
    std::sort(selectedLanelets.begin(), selectedLanelets.end(), [](const std::pair<int, lanelet::Lanelet>& a, std::pair<int, lanelet::Lanelet>& b) {return a.first < b.first;});
    std::vector<lanelet::Lanelet> allLanelets;
    for(auto& pair: selectedLanelets)
      allLanelets.push_back(pair.second);
    
    allLanelets.push_back(firstLanelet);
    
    //Find all the right lanelets
    currentLanelet = firstLanelet;
    lanelets = std::get<2>(nearLanelets);
    count = 0;
    selectedLanelets.clear();
    while(true){
      auto result = findTheRightLanelet(currentLanelet, lanelets);
      if(result.first){
        allLanelets.push_back(result.second);
        currentLanelet = result.second;
        lanelet::ConstLineString3d centerline = currentLanelet.centerline();  
        int midSegNo = centerline.numSegments()/2;
        auto midSeg = centerline.segment(midSegNo);
        auto point1 = midSeg.first;auto point2 = midSeg.second;
        auto midX = (point1.x()+point2.x())/2; auto midY = (point1.y()+point2.y())/2;
        auto point = lanelet::BasicPoint2d(midX, midY);
        auto otherLanelets = findTheActualLanelet(map, point);
        lanelets = std::get<2>(otherLanelets);
        count++;
      }else
        break;
    }

    //Find the point in the left linestring of the left most lanelet
    auto lsLeft = allLanelets[0].leftBound();

    //Find the points in the right linestring of the rightmost lanelet where we
    //can project the point
    auto lsRight = allLanelets[allLanelets.size()-1].rightBound();
   
    //Find the distance between the two points that is considered as the width
    //of the road
    lanelet::Lanelet boundaryLanelet(lanelet::utils::getId(), lsLeft, lsRight);
    if(allLanelets.size()%2 == 0){
      //even then, find the two middle two lanelets and get the commly shared
      //linestring as the center point.
      int rem = allLanelets.size()/2;
      //if(rem == 1)
      //  rem = 0; 
      rem -=1;
      auto firstLL = allLanelets[rem];
      auto centerLine = firstLL.rightBound();
      if(centerLine.size() > 0){
        auto pProj = lanelet::geometry::project(centerLine, lanelet::Point3d{lanelet::utils::getId(), startingPoint.x(), startingPoint.y()});
        _return = std::make_tuple(true, std::make_pair(lanelet::BasicPoint2d(pProj.x(), pProj.y()),centerLine), std::make_pair(boundaryLanelet, allLanelets.size()));
      }
    }else{
      //For odd number
      int rem = allLanelets.size()/2;
      if(rem < 1){
        auto lanelet = allLanelets[0];
        auto centerLine = lanelet.centerline();
        auto pProj = lanelet::geometry::project(centerLine, lanelet::Point3d{lanelet::utils::getId(), startingPoint.x(), startingPoint.y()});
        lanelet::Point3d p1{lanelet::utils::getId(), centerLine.front().x(), centerLine.front().y(), 0};
        lanelet::Point3d p2{lanelet::utils::getId(), centerLine.back().x(), centerLine.back().y(), 0};
        lanelet::LineString3d ls3d(lanelet::utils::getId(),{p1, p2});
        _return = std::make_tuple(true, std::make_pair(lanelet::BasicPoint2d(pProj.x(), pProj.y()),ls3d), std::make_pair(boundaryLanelet,allLanelets.size()));

      }else{
        //auto lanelet = allLanelets[rem+1];
        auto lanelet = allLanelets[rem];
        auto centerLine = lanelet.centerline();
        auto pProj = lanelet::geometry::project(centerLine, lanelet::Point3d{lanelet::utils::getId(), startingPoint.x(), startingPoint.y()});
        lanelet::Point3d p1{lanelet::utils::getId(), centerLine.front().x(), centerLine.front().y(), 0};
        lanelet::Point3d p2{lanelet::utils::getId(), centerLine.back().x(), centerLine.back().y(), 0};
        lanelet::LineString3d ls3d(lanelet::utils::getId(),{p1, p2});
        _return = std::make_tuple(true, std::make_pair(lanelet::BasicPoint2d(pProj.x(), pProj.y()),ls3d), std::make_pair(boundaryLanelet,allLanelets.size()));
      }
    }
     
  }//Checking lanelet for the egopoint if closes      
  else{
    //ROS_INFO_STREAM("No lanelet found: length of the lanelet is zero or simply couldn't find the lanelet that actually has the point");
  }

  return _return;
}

*/

void frenetJsonVec(std::vector<std::pair<int, std::vector<json>>>& frenetJson, std::tuple<int, json, json> data, lanelet::BasicPoint2d roadCenter, double s, json otherJdata, lanelet::BasicPoint2d egoCenter){
 
  auto car = std::get<0>(data);
  json mainJData = {
    {"car_id", car},
    {"frenet_data", std::get<1>(data)},
    {"odom_pos", std::get<2>(data)},
    {"road_center_x", roadCenter.x()},
    {"road_center_y", roadCenter.y()},
    //{"ego_center_x", egoCenter.x()},
    //{"ego_center_y", egoCenter.y()},
    {"s", s},
    {"other", otherJdata}
  };
  bool found = false;
  size_t index = 0;
  for(size_t i=0; i<frenetJson.size(); i++){
    auto pair = frenetJson[i];
    if(pair.first == car){
      found = true;
      index = i;
    }
  }
  if(found){
    auto pair = frenetJson[index]; 
    auto vec = pair.second;
    vec.push_back(mainJData);
    frenetJson[index] = std::make_pair(pair.first, vec);
  }else{
    std::vector<json> tempVec; tempVec.push_back(mainJData);
    frenetJson.push_back(std::make_pair(car, tempVec));
  }
}

void frenetJsonVecEgo(std::vector<json>& frenetJsonEgo, std::pair<json, json> data, lanelet::BasicPoint2d roadCenter, double s, json otherJdata){
 
  json mainJData = {
    {"frenet_data", data.first},
    {"odom_pos", data.second},
    {"road_center_x", roadCenter.x()},
    {"road_center_y", roadCenter.y()},
    {"s", s},
    {"other", otherJdata}
  };
  
  frenetJsonEgo.push_back(mainJData);
}


double getFrenetSEgoRef(std::vector<std::pair<int, std::vector<json>>> frenetJsonOtherEgoRef, int car, lanelet::LineString3d egoCenterLine, lanelet::Point3d point, lanelet::BasicPoint2d egoCenter){
  auto closestSeg = lanelet::geometry::closestSegment(egoCenterLine, point);
  double s = 0.;
  bool found = false;
  size_t index = 0;
  for(size_t i=0; i<frenetJsonOtherEgoRef.size(); i++){
    auto pair = frenetJsonOtherEgoRef[i];
    if(pair.first == car){
      found = true;
      index = i;
    }
  }
  if(found){
    auto pair = frenetJsonOtherEgoRef[index]; 
    auto vec = pair.second;
    json jData = vec.back();
    auto prevS = jData["s"];

    s = jData["s"].get<double>()+std::sqrt(std::pow((jData["ego_center_x"].get<double>()-egoCenter.x()),2)+std::pow((jData["ego_center_y"].get<double>()-egoCenter.y()),2));
  }else{
    lanelet::Point3d p1{lanelet::utils::getId(), egoCenterLine.front().x(), egoCenterLine.front().y(), 0};
    lanelet::Point3d p2{lanelet::utils::getId(), egoCenter.x(), egoCenter.y(), 0};
    lanelet::LineString3d ls(lanelet::utils::getId(),{p1, p2});
    s = lanelet::geometry::length(lanelet::utils::toHybrid(ls));
  }
  
  return s;
}
  
double getFrenetS(std::vector<std::pair<int, std::vector<json>>> frenetJson, int car, lanelet::LineString3d roadCenterLine, lanelet::Point3d point, lanelet::BasicPoint2d roadCenter){

  auto closestSeg = lanelet::geometry::closestSegment(roadCenterLine, point);
  double s = 0.;
  bool found = false;
  size_t index = 0;
  for(size_t i=0; i<frenetJson.size(); i++){
    auto pair = frenetJson[i];
    if(pair.first == car){
      found = true;
      index = i;
    }
  }
  if(found){
    auto pair = frenetJson[index]; 
    auto vec = pair.second;
    json jData = vec.back();
    auto prevS = jData["s"];

    s = jData["s"].get<double>()+std::sqrt(std::pow((jData["road_center_x"].get<double>()-roadCenter.x()),2)+std::pow((jData["road_center_y"].get<double>()-roadCenter.y()),2));
  }else{
    lanelet::Point3d p1{lanelet::utils::getId(), roadCenterLine.front().x(), roadCenterLine.front().y(), 0};
    lanelet::Point3d p2{lanelet::utils::getId(), roadCenter.x(), roadCenter.y(), 0};
    lanelet::LineString3d ls(lanelet::utils::getId(),{p1, p2});
    s = lanelet::geometry::length(lanelet::utils::toHybrid(ls));
  }
  
  return s;
}

double getFrenetSEgo(std::vector<json>& frenetJsonEgo, lanelet::LineString3d roadCenterLine, lanelet::BasicPoint2d roadCenter){

  double s = 0.;
  if(frenetJsonEgo.size() > 0){
    auto jData = frenetJsonEgo.back();
    s = jData["s"].get<double>()+std::sqrt(std::pow((jData["road_center_x"].get<double>()-roadCenter.x()),2)+std::pow((jData["road_center_y"].get<double>()-roadCenter.y()),2));
  }else{
    lanelet::Point3d p1{lanelet::utils::getId(), roadCenterLine.front().x(), roadCenterLine.front().y(), 0};
    lanelet::Point3d p2{lanelet::utils::getId(), roadCenter.x(), roadCenter.y(), 0};
    lanelet::LineString3d ls(lanelet::utils::getId(),{p1, p2});
    s = lanelet::geometry::length(lanelet::utils::toHybrid(ls));
  }
  
  return s;
}



bool checkObjectSecAdded(std::vector<std::pair<int, std::vector<json>>> frenetJson, int car, uint32_t sec){
  bool found = false;
  for(size_t i=0; i<frenetJson.size(); i++){
    auto pair = frenetJson[i];
    if(pair.first == car){
      auto vec = pair.second;
      for(auto& j: vec){
        if(j["frenet_data"]["sec"].get<uint32_t>() == sec){
          found = true;
          break;
        }
      }
    }
  }

  return found;
} 

lanelet::LineString3d getTheRoadLineString(lanelet::LineString3d roadCenterLine, lanelet::Point3d point)
{
  auto seg = lanelet::geometry::closestSegment(roadCenterLine, point);
  lanelet::LineString3d _returnls(lanelet::utils::getId(), {seg.first, seg.second});
  
  /*
  for(size_t i=0; i<roadCenterLine.numSegments(); i++){
    auto seg = roadCenterLine.segment(i);
    lanelet::LineString3d ls(lanelet::utils::getId(), {seg.first, seg.second});
    auto pointBox = lanelet::geometry::boundingBox3d(point);
    auto lsBox = lanelet::geometry::boundingBox3d(ls);
    if(lanelet::geometry::intersects(pointBox, lsBox)){
      _returnls.push_back(seg.first);
      _returnls.push_back(seg.second);
      break;
    }
  }*/

  return _returnls;
}

std::pair<bool, lanelet::Lanelet> findTheClosestLaneletEgo(lanelet::LaneletMapPtr map, lanelet::BasicPoint2d startingPoint){
  std::vector<std::pair<double, lanelet::Lanelet>> nearLanelets = lanelet::geometry::findNearest(map->laneletLayer, startingPoint, 10);
  std::vector<lanelet::Lanelet> lanelets;
  lanelet::Lanelet lanelet;
  double smallD = 1000.; size_t selectedIndex = 0; bool found = false;
  for(size_t i=0; i<nearLanelets.size(); i++){
    if(nearLanelets[i].first < smallD && nearLanelets[i].first < 10){
      smallD = nearLanelets[i].first;
      selectedIndex = i;
      found = true;
    }
    lanelets.push_back(nearLanelets[i].second);
  }
  if(found){
    lanelet = lanelets[selectedIndex];
  }

  return std::make_pair(found, lanelet);
}

bool checkPointExistInLineString(lanelet::LineString3d roadCenterLine, lanelet::Point3d checkPoint){
  bool found = false;
  for(auto& point: roadCenterLine){
    if(point.x() == checkPoint.x() && point.y() == checkPoint.y()){
      found = true;
      break;
    }
  }

  return found;
}

//https://www.geeksforgeeks.org/direction-point-line-segment/
const int RIGHT = 1, LEFT = -1, ZERO = 0;
int directionOfPoint(lanelet::Point3d A, lanelet::Point3d B, lanelet::BasicPoint2d P)
{
    // subtracting co-ordinates of point A from
    // B and P, to make A as origin
    B.x() -= A.x();
    B.y() -= A.y();
    P.x() -= A.x();
    P.y() -= A.y();
 
    // Determining cross Product
    int cross_product = B.x() * P.y() - B.y() * P.x();
 
    // return RIGHT if cross product is positive
    if (cross_product > 0)
        return RIGHT;
 
    // return LEFT if cross product is negative
    if (cross_product < 0)
        return LEFT;
 
    // return ZERO if cross product is zero.
    return ZERO;
}

bool checkSecObjExist(int object_id, uint32_t sec, uint32_t nsec, std::vector<std::tuple<int, uint32_t, int>>& objSecVec, int threshold=2){
  bool proceed = false;
  bool found = false;
  for(size_t i=0; i<objSecVec.size(); i++){
    auto tuple = objSecVec[i];
    auto id = std::get<0>(tuple);
    auto s = std::get<1>(tuple);
    if(id == object_id && s == sec){
      found = true;
      auto count = std::get<2>(tuple);
      count += 1;
      objSecVec[i] = std::make_tuple(id, s, count);
      if(count%threshold == 0){
        proceed = true;
      }
      break;
    }
  }

  if(!found){
      objSecVec.push_back(std::make_tuple(object_id, sec, 1));
      proceed = true;
  }
 

  return proceed;
}


#endif //APPLICATIONS_HELPER_FUNCTIONS_HPP
