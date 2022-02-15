#ifndef APPLICATIONS_HELPER_FUNCTIONS_HPP
#define APPLICATIONS_HELPER_FUNCTIONS_HPP

std::string tempfile(const std::string& name) {
  char tmpDir[] = "/tmp/lanelet2_example_XXXXXX";
  auto* file = mkdtemp(tmpDir);
  if (file == nullptr) {
    throw lanelet::IOError("Failed to open a temporary file for writing");
  }

  ROS_INFO_STREAM("File created: "<<std::string(file));

  return std::string(file) + '/' + name;
}

std::tuple<double, double, double> linearRegression(std::vector<double> x, std::vector<double> y){
  //Linear regression based on least square method
  //Reference doc: https://www.mathsisfun.com/data/least-squares-regression.html  
  auto N = x.size();
  double sumXY = 0;
  double sumX = 0;
  double sumY = 0;
  double sumXSquared = 0;
  for(size_t i=0; i<N; i++){

    auto xy = x[i]*y[i];
    sumXY += xy;
    sumX += x[i];
    sumY += y[i];
    auto xSquared = std::pow(x[i], 2);
    sumXSquared += xSquared;
  }
  double m = 0.;
  double c = 0.;
  m =  ((N*sumXY) - (sumX*sumY))/((N*sumXSquared)-(std::pow(sumX, 2)));
  c = (sumY-(m*sumX))/N;
  //Mean squared error
  double mse = 0.;
  for(size_t i=0; i<N; i++){
    auto y_pred = m*x[i]+c; // y = mx+c form
    mse += std::pow((y[i] - y_pred),2);
  }
  mse = mse/N;

  return std::make_tuple(m, c, mse);
}


void joinLineString3d(std::vector<std::pair<lanelet::LineString3d, int>>& activeLineString3d, std::vector<lanelet::LineString3d>& allLineStrings3d, lanelet::LineString3d ls3d){
   
  auto seg2 = ls3d.segment(ls3d.numSegments()-1);
  auto length = lanelet::geometry::distance(seg2.first, seg2.second);
  if(length > 0.5){
    std::vector<double> xs; std::vector<double> ys;
    xs.push_back(seg2.first.x());xs.push_back(seg2.second.x());
    ys.push_back(seg2.first.y());ys.push_back(seg2.second.y());
    auto lineDetails2 = linearRegression(xs, ys);
    if(activeLineString3d.size() == 0){
      activeLineString3d.push_back(std::make_pair(ls3d, 0));
    }else{ 
      double smallD = 10000.;
      size_t selectedIndex = 0;
      bool enable = false;
      for(size_t i = 0; i<activeLineString3d.size(); i++){
        auto line3d = activeLineString3d[i].first;
        auto inactive = activeLineString3d[i].second;
        int count = 0;
        std::vector<double> xs; std::vector<double> ys;
        if(line3d.numSegments() == 1){
          auto seg1 = line3d.segment(0);
          xs.push_back(seg1.first.x());xs.push_back(seg1.second.x());
          ys.push_back(seg1.first.y());ys.push_back(seg1.second.y());
        }else{
          auto seg1 = line3d.segment(line3d.numSegments()-1);
          xs.push_back(seg1.first.x());xs.push_back(seg1.second.x());
          ys.push_back(seg1.first.y());ys.push_back(seg1.second.y());
          seg1 = line3d.segment(line3d.numSegments()-2);
          xs.push_back(seg1.first.x());xs.push_back(seg1.second.x());
          ys.push_back(seg1.first.y());ys.push_back(seg1.second.y());
        }
        auto lineDetails1 = linearRegression(xs, ys);
        auto seg1 = line3d.segment(line3d.numSegments()-1);
        auto slopeDiff = std::abs(std::get<0>(lineDetails2) - std::get<0>(lineDetails1));
        auto y = std::get<0>(lineDetails1) * seg2.first.x()+ std::get<1>(lineDetails1);
        auto y_diff = std::abs(std::abs(y) - std::abs(seg2.first.y()));
        auto d = lanelet::geometry::distance(seg1.second, seg2.first);
        if(slopeDiff < 0.1 && y_diff < 0.2 && y_diff < smallD && d<50){
          smallD = y_diff;
          selectedIndex = i;
          enable = true;
          break;
        }
      }

      if(enable){
        for(size_t i = 0; i<activeLineString3d.size(); i++){
          if(i == selectedIndex){
            auto line3d = activeLineString3d[i].first;
            auto seg1 = line3d.segment(line3d.numSegments()-1);
            xs.push_back(seg1.first.x());xs.push_back(seg1.second.x());
            ys.push_back(seg1.first.y());ys.push_back(seg1.second.y());
            auto lineDetails1 = linearRegression(xs, ys);
            xs.clear();ys.clear();
            xs.push_back(seg1.second.x());xs.push_back(seg2.first.x());
            ys.push_back(seg1.second.y());ys.push_back(seg2.first.y());
            auto lineDetails3 = linearRegression(xs, ys);
            auto slopeDiff = std::abs(std::get<0>(lineDetails3) - std::get<0>(lineDetails1));
            if(slopeDiff < 0.1){
              line3d.push_back(seg2.first);line3d.push_back(seg2.second);
              activeLineString3d[i] = std::make_pair(line3d, 0);
            }else{
              activeLineString3d.push_back(std::make_pair(ls3d, 0));
            } 
            //line3d.push_back(seg2.first);line3d.push_back(seg2.second);
            //activeLineString3d[i] = std::make_pair(line3d, 0);

          }else{
            auto line3d = activeLineString3d[i].first;
            auto inactive = activeLineString3d[i].second;
            inactive += 1;
            if(inactive > 5){
              allLineStrings3d.push_back(line3d);            
              activeLineString3d.erase(activeLineString3d.begin()+i);
            }else{
              activeLineString3d[i] = std::make_pair(line3d, inactive);
            }
          }
        }
      }else{
        activeLineString3d.push_back(std::make_pair(ls3d, 0));
      }
    }
  }
}

bool checkIntersectionOdomAndConnectionLine(std::pair<double, double> firstPoint, std::pair<double, double> secondPoint, Segment seg){
 
  //Finding the new line intersects with ODOM or not
  lanelet::Point3d p1{lanelet::utils::getId(), firstPoint.first, firstPoint.second, 0};
  lanelet::Point3d p2{lanelet::utils::getId(), secondPoint.first, secondPoint.second, 0};
  lanelet::LineString3d ls3d1(lanelet::utils::getId(), {p1, p2});
  auto lsBox1 = lanelet::geometry::boundingBox3d(ls3d1);
  lanelet::Point3d p3{lanelet::utils::getId(), bg::get<0, 0>(seg), bg::get<0, 1>(seg), 0};
  lanelet::Point3d p4{lanelet::utils::getId(), bg::get<1, 0>(seg), bg::get<1, 1>(seg), 0};
  lanelet::LineString3d ls3d2(lanelet::utils::getId(), {p3, p4});
  auto lsBox2 = lanelet::geometry::boundingBox3d(ls3d2);

  return lanelet::geometry::intersects(lsBox1, lsBox2);
}

lanelet::LineString3d getlaneletLineStringFromBoostLineString(LineString ls){
  auto p1 = ls.front();
  auto p2 = ls.back();
  lanelet::Point3d p3d1{lanelet::utils::getId(), p1.get<0>(), p1.get<1>(), 0};
  lanelet::Point3d p3d2{lanelet::utils::getId(), p2.get<0>(), p2.get<1>(), 0};
  lanelet::LineString3d ls3d(lanelet::utils::getId(), {p3d1, p3d2});

  return ls3d;
}

lanelet::LineString3d convertLineStringToLineString3d(LineString ls){
  lanelet::LineString3d ls3d(lanelet::utils::getId(),{});
  for(size_t i=0; i<ls.size(); i++){
    lanelet::Point3d p(lanelet::utils::getId(), {ls[i].get<0>(), ls[i].get<1>(), 0});
    ls3d.push_back(p); 
  }

  return ls3d; 
}


void RemoveCloserLinesOnLeftOrRight(std::vector<LineString>& allLineStrings){
  std::vector<size_t> eraseVec;
  double distT = 1.5;
  for(size_t i=0; i<allLineStrings.size(); i++){
    if(std::find(eraseVec.begin(), eraseVec.end(), i) != eraseVec.end())
      continue; 
    auto ls3d1 = convertLineStringToLineString3d(allLineStrings[i]);    
    auto d1 = lanelet::geometry::length(lanelet::utils::toHybrid(ls3d1));
    int end = i+5;
    if((i+5) > allLineStrings.size())
      end = allLineStrings.size();
    for(size_t j=i+1; j<end; j++){
      auto ls3d2 = convertLineStringToLineString3d(allLineStrings[j]); 
      auto d2 = lanelet::geometry::length(lanelet::utils::toHybrid(ls3d2));
      if(std::find(eraseVec.begin(), eraseVec.end(), j) != eraseVec.end() || i==j)
        continue;
      //The longer length means, higher chance of having proper lane
      if(d1 > d2){
        auto p1 = ls3d2.front();auto p2 = ls3d2.back();
        auto pProj1 = lanelet::geometry::project(ls3d2, p1);
        auto pProj2 = lanelet::geometry::project(ls3d2, p2);
        lanelet::BasicPoint2d point1(pProj1.x(), pProj1.y());
        lanelet::BasicPoint2d point2(pProj2.x(), pProj2.y());
        auto dist1 = lanelet::geometry::signedDistance(lanelet::utils::to2D(ls3d1), point1);
        auto dist2 = lanelet::geometry::signedDistance(lanelet::utils::to2D(ls3d1), point2);
        if(std::abs(dist1) < distT && std::abs(dist2) < distT){
          eraseVec.push_back(j);
        }
      }else{
        /*auto p1 = ls3d1.front();auto p2 = ls3d1.back();
        auto pProj1 = lanelet::geometry::project(ls3d1, p1);
        auto pProj2 = lanelet::geometry::project(ls3d1, p2);
        lanelet::BasicPoint2d point1(pProj1.x(), pProj1.y());
        lanelet::BasicPoint2d point2(pProj2.x(), pProj2.y());
        auto dist1 = lanelet::geometry::signedDistance(lanelet::utils::to2D(ls3d2), point1);
        auto dist2 = lanelet::geometry::signedDistance(lanelet::utils::to2D(ls3d2), point2);
        if(std::abs(dist1) < distT || std::abs(dist2) < distT){
          eraseVec.push_back(i);
          ROS_INFO_STREAM(std::abs(dist1)<<" "<<std::abs(dist2)<<" i:"<<i);
        }*/
      }
    }
  }

  for(size_t i=0; i<eraseVec.size(); i++){
    allLineStrings.erase(allLineStrings.begin()+eraseVec[i]);
  }
}

std::tuple<double, double, double>  applyLinearRegressionOnLS(lanelet::LineString3d ls3d, std::string type){
  std::vector<double> xs; std::vector<double> ys;
  /*if(type == "back"){
    int end = 10;
    if((ls3d.size()-10) < end)
      end = ls3d.size();
    for(int i = ls3d.size()-1; i>=end; i--){
      auto p = ls3d[i];
      xs.push_back(p.x());
      ys.push_back(p.y());
    }
  }else{
    int end = 10;
    if(ls3d.size() < 10)
     end = ls3d.size();
    for(int i=0; i<end; i++){
      auto p = ls3d[i];
      xs.push_back(p.x());
      ys.push_back(p.y());
    }
  }*/

  if(type == "front"){
    for(int i=0; i<ls3d.numSegments(); i++){
      auto seg =ls3d.segment(i);
      xs.push_back(seg.first.x());
      ys.push_back(seg.first.y());
      xs.push_back(seg.second.x());
      ys.push_back(seg.second.y());
      if(i == 6)
        break;
    }
  }else{
    int count = 0;
    for(int i=ls3d.numSegments()-1; i>=0; i--){
      auto seg =ls3d.segment(i);
      xs.push_back(seg.first.x());
      ys.push_back(seg.first.y());
      xs.push_back(seg.second.x());
      ys.push_back(seg.second.y());
      if(count == 6)
        break;
      count++;
    }
  }
  
  return linearRegression(xs, ys);
}

std::tuple<double, double, double> applyLinearRegressionOnPoint(Point p1, Point p2){
  std::vector<double> xs; std::vector<double> ys;
  xs.push_back(p1.get<0>());
  xs.push_back(p2.get<0>()); 
  ys.push_back(p1.get<1>());
  ys.push_back(p2.get<1>());

  return linearRegression(xs, ys);
}


double findTheAngleBWTwoLines(double M1, double M2){

  // Store the tan value  of the angle
  double angle = std::abs((M2 - M1)/ (1 + M1 * M2));
  
  // Calculate tan inverse of the angle
  double ret = atan(angle);
 
  // Convert the angle from
  // radian to degree
  double val = (ret * 180) / 3.14159265;
  
  return val;
}

void joincleanLineSegements(std::vector<LineString>& allLineStrings, double angleT, double distT, double smallDistT){
  std::vector<size_t> eraseVec;
  std::vector<size_t> deleteVec;
  for(size_t i=0; i<allLineStrings.size(); i++){
    if(std::find(eraseVec.begin(), eraseVec.end(), i) != eraseVec.end())
      continue;
    auto ls3d1 = convertLineStringToLineString3d(allLineStrings[i]);  
    auto lineDetails1 = applyLinearRegressionOnLS(ls3d1,"front");
    auto dist = lanelet::geometry::length(lanelet::utils::toHybrid(ls3d1));
    if(dist < smallDistT){
      eraseVec.push_back(i);
      continue;
    }
    auto p1 = ls3d1.front(); 
    std::vector<std::tuple<size_t, double, double>> storeVec;
    for(size_t j=i+1; j<allLineStrings.size(); j++){
      if(std::find(eraseVec.begin(), eraseVec.end(), j) != eraseVec.end() || i == j)
        continue;
      
      if(dist < 10.){
        eraseVec.push_back(j);
        continue;
      }

      auto ls3d2 = convertLineStringToLineString3d(allLineStrings[j]);
      auto p2 = ls3d2.back(); 
        lanelet::LineString3d newLs3d(lanelet::utils::getId(), {p1, p2});
        auto lineDetails2 = applyLinearRegressionOnLS(newLs3d,"back");
        auto d = lanelet::geometry::length(lanelet::utils::toHybrid(newLs3d));
        double angle = findTheAngleBWTwoLines(std::get<0>(lineDetails1), std::get<0>(lineDetails2));
        if(angle < angleT && d<distT){
          storeVec.push_back(std::make_tuple(j, angle, d));
        }
    }

    std::sort(storeVec.begin(), storeVec.end(), [](const std::tuple<size_t, double, double>& a, const std::tuple<size_t, double, double>& b) {return std::get<2>(a) < std::get<2>(b);});
    if(storeVec.size() > 0){
      auto vec = storeVec[0];
      auto selectedIndex = std::get<0>(vec);
      auto ls = allLineStrings[selectedIndex];
      for(auto& p: ls){
       allLineStrings[i].push_back(p);
      }
      eraseVec.push_back(selectedIndex);
      deleteVec.push_back(selectedIndex);
    }
    eraseVec.push_back(i);
  }
  
  /*for(size_t i=0;i<deleteVec.size(); i++){
    allLineStrings.erase(allLineStrings.begin()+deleteVec[i]);   
  }*/
}

void checkIntersect(std::vector<LineString>& allLineStrings){
  std::vector<size_t> eraseVec;
  for(size_t i=0; i<allLineStrings.size(); i++){
    if(std::find(eraseVec.begin(), eraseVec.end(), i) != eraseVec.end())
      continue;
    auto ls3d1 = convertLineStringToLineString3d(allLineStrings[i]);  
    auto p1 = ls3d1.front(); 
    auto p2 = ls3d1.back(); 
    auto lsBox1 = lanelet::geometry::boundingBox3d(ls3d1);
    for(size_t j=0; j<allLineStrings.size(); j++){
      if(i == j)
        continue;
      auto ls3d2 = convertLineStringToLineString3d(allLineStrings[j]);
      auto p3 = ls3d2.front(); 
      auto p4 = ls3d2.back(); 
      auto pBox3 = lanelet::geometry::boundingBox2d(p3);
      auto pBox4 = lanelet::geometry::boundingBox2d(p4);
      lanelet::LineString3d newLs3d(lanelet::utils::getId(), {p1, p4});
      auto d = lanelet::geometry::length(lanelet::utils::toHybrid(newLs3d));
      MultiPoint interPoints1; MultiPoint interPoints2;
      bg::intersection(allLineStrings[j].front(), allLineStrings[i], interPoints1);
      bg::intersection(allLineStrings[j].back(), allLineStrings[i], interPoints2);
      if(interPoints1.size() > 0 && interPoints2.size() > 0){
        auto d1 = bg::length(allLineStrings[i]);
        auto d2 = bg::length(allLineStrings[j]);
        if(d2 < 50){
          eraseVec.push_back(j);
        }
      }
    }
  } 
  sort(eraseVec.begin(), eraseVec.end() );
  eraseVec.erase(unique(eraseVec.begin(), eraseVec.end()), eraseVec.end());
  for(size_t i=0;i<eraseVec.size(); i++){
    allLineStrings.erase(allLineStrings.begin()+eraseVec[i]);   
  }

}

std::pair<double, double> getPerpendicularLineDetails(std::tuple<double, double, double> odomLineDetails, Point p1){
  auto m = std::get<0>(odomLineDetails);
  auto M2 = (1/m)*-1;
  auto C2 = p1.get<1>() - (M2*p1.get<0>());
 
  return std::make_pair(M2, C2);
}

/*std::pair<lanelet::LineString3d, LineString> getLineFromDetails(std::pair<double, double> lineDetails, Point p){
  auto dist = 9.;
  auto y1 = p.get<1>()+dist;
  auto x1 = y1-lineDetails.second/lineDetails.first; 
  auto x2 = p.get<0>();
  auto y2 = p.get<1>();
  dist = -9.;
  auto y3= p.get<1>()+dist;
  auto x3 = y3-lineDetails.second/lineDetails.first;
  LineString ls;
  ls.push_back(Point(x1, y1));
  ls.push_back(Point(x2, y2));
  ls.push_back(Point(x3, y3));
  lanelet::Point3d p3d1{lanelet::utils::getId(), x1, y1, 0};
  lanelet::Point3d p3d2{lanelet::utils::getId(), x2, y2, 0};
  lanelet::Point3d p3d3{lanelet::utils::getId(), x3, y3, 0};
  lanelet::LineString3d ls3d(lanelet::utils::getId(), {p3d1, p3d3});
  
  return std::make_pair(ls3d, ls);
}*/

std::pair<lanelet::LineString3d, LineString> getLineFromDetails(Point p1, Point p2, std::string type){
  //https://www.codeproject.com/Questions/1067954/How-to-Find-points-for-draw-a-Perpendicular-line-o
  double N = 12.;
  auto x1 = p1.get<0>(); auto y1 = p1.get<1>();
  auto x2 = p2.get<0>(); auto y2 = p2.get<1>();
  auto dx = x1-x2;
  auto dy = y1-y2;
  auto dist = std::sqrt(dx*dx + dy*dy);
  dx /= dist;
  dy /= dist;
  double x3; double y3; double x4; double y4; 
  if(type == "first") {
    x3 = x1 + (N/2)*dy; // where N is the length of line to be draw.
    y3 = y1 - (N/2)*dx;
    x4 = x1 - (N/2)*dy;
    y4 = y1 + (N/2)*dx;
  }else{
    x3 = x2 + (N/2)*dy; // where N is the length of line to be draw.
    y3 = y2 - (N/2)*dx;
    x4 = x2 - (N/2)*dy;
    y4 = y2 + (N/2)*dx;
  }

  LineString ls;
  ls.push_back(Point(x3, y3));
  ls.push_back(Point(x4, y4));
  lanelet::Point3d p3d3{lanelet::utils::getId(), x3, y3, 0};
  lanelet::Point3d p3d4{lanelet::utils::getId(), x4, y4, 0};
  lanelet::LineString3d ls3d(lanelet::utils::getId(), {p3d3, p3d4});
  
  return std::make_pair(ls3d, ls);

}

Point findInterpolatedPointGivenX(double x0, double y0, double x1, double y1, double xp){

  /* Linear Interpolation */
  auto yp = y0 + ((y1-y0)/(x1-x0)) * (xp - x0);
  
  return Point(xp, yp);
}

lanelet::Point3d pointToPoint3d(Point p){
  lanelet::Point3d point3d{lanelet::utils::getId(), p.get<0>(), p.get<1>(), 0};

  return point3d;
}

bool checkLineStrings3d(std::vector<lanelet::LineString3d> lineStrings3d, LineString ls){
  bool _return = false;
  for(auto& ls3d: lineStrings3d){
    auto x1 = ls3d.front().x();auto y1 = ls3d.front().y();auto x2 = ls3d.back().x();auto y2 = ls3d.back().y();
    auto x = (x1+x2)/2;auto y = (y1+y2)/2;
    auto pProj = lanelet::geometry::project(convertLineStringToLineString3d(ls), pointToPoint3d(Point(x,y)));
    lanelet::LineString3d ls3dTemp(lanelet::utils::getId(), {pointToPoint3d(Point(x,y)),pointToPoint3d(Point(pProj.x(),pProj.y()))});
    auto d = lanelet::geometry::length(lanelet::utils::toHybrid(ls3dTemp));
    if(d < 2.){
      _return = true;
      break; 
    }
  }

  return _return;
}

std::vector<lanelet::LineString3d> createLineStringsFromInterPoints(LineString odomLS, std::vector<std::pair<Point, LineString>> interPoints1, std::vector<std::pair<Point, LineString>> interPoints2, Point oPoint1, Point oPoint2){
  
  std::vector<LineString> lineStrings;
  std::vector<lanelet::LineString3d> lineStrings3d;
  std::vector<std::pair<std::pair<Point, LineString>, std::pair<Point, LineString>>> keepTrackL;
  std::vector<std::pair<std::pair<Point, LineString>, std::pair<Point, LineString>>> keepTrackAll;

  std::vector<double>odomXs;std::vector<double>odomYs;
  odomXs.push_back(oPoint1.get<0>());odomXs.push_back(oPoint2.get<0>());
  odomYs.push_back(oPoint1.get<1>());odomYs.push_back(oPoint2.get<1>());
  auto odomLineDetails = linearRegression(odomXs, odomYs);

  //Join the points to create linestring that has lowest line angle and only
  //first end point is there if there is no lowest line angle
  for(size_t i=0; i<interPoints1.size(); i++){
    auto p1 = interPoints1[i].first;
    double smallA = 1000.;
    size_t selectedIndex = 0;
    double selectedAngle = 1000.;
    for(size_t j=0; j<interPoints2.size(); j++){
      auto p2 = interPoints2[j].first;
      double angle = atan2(p2.get<1>() - p1.get<1>(), p2.get<0>() - p1.get<0>()) * 180 / 3.14159265; 
      if(std::abs(angle) < smallA){
        //Finding the line points with smallest angle 
        smallA = std::abs(angle);
        selectedIndex = j;
        selectedAngle = std::abs(angle);
      }
    }//For loop of j closes
    if(selectedAngle < 1){
      //There are both end points
      LineString ls;ls.push_back(p1);ls.push_back(interPoints2[selectedIndex].first);
      if(!checkLineStrings3d(lineStrings3d, ls)){
        std::vector<double>xs;std::vector<double>ys;
        xs.push_back(p1.get<0>());xs.push_back(interPoints2[selectedIndex].first.get<0>());
        ys.push_back(p1.get<1>());ys.push_back(interPoints2[selectedIndex].first.get<1>());
        auto lineDetails = linearRegression(xs, ys);
        auto slopeDiff = std::abs(std::get<0>(lineDetails)-std::get<0>(odomLineDetails));
        if(slopeDiff < 0.1 && lanelet::geometry::length(lanelet::utils::toHybrid(convertLineStringToLineString3d(ls))) < 27) 
          lineStrings3d.push_back(convertLineStringToLineString3d(ls));
      }
    }else{
      //There is no second end point
     // In some case you have only one point, so it would be hard to construct a linestring. The solution is is to find the end point of that line, then get the slope equation form of that line to find out the end point upto the x of the odom point..
      Point secondPoint = interPoints1[i].second.front();
      Point endPoint = findInterpolatedPointGivenX(p1.get<0>(), p1.get<1>(), secondPoint.get<0>(), secondPoint.get<1>(), oPoint2.get<0>());
      LineString ls;ls.push_back(p1);ls.push_back(endPoint);
      if(!std::isnan(endPoint.get<1>())){
        LineString lsTemp; 
        if(keepTrackL.size() == 0){
          std::vector<double>xs;std::vector<double>ys;
          xs.push_back(p1.get<0>());xs.push_back(endPoint.get<0>());
          ys.push_back(p1.get<1>());ys.push_back(endPoint.get<1>());
          auto lineDetails = linearRegression(xs, ys);
          auto slopeDiff = std::abs(std::get<0>(lineDetails)-std::get<0>(odomLineDetails));
          if(slopeDiff < 0.1 && lanelet::geometry::length(lanelet::utils::toHybrid(convertLineStringToLineString3d(ls))) < 27){
            keepTrackL.push_back(std::make_pair(std::make_pair(p1, interPoints1[i].second), std::make_pair(endPoint, lsTemp)));
            lineStrings3d.push_back(convertLineStringToLineString3d(ls));
          }
        }else{
          size_t count = 0 ;
          bool proceed = true;
          for(auto pair: keepTrackL){
            auto pointPair1 = pair.first; auto pointPair2 = pair.second;
            auto xdiff1 = std::abs(pointPair1.first.get<0>() - p1.get<0>());
            auto ydiff1 = std::abs(pointPair1.first.get<1>() - p1.get<1>());
            auto xdiff2 = std::abs(pointPair2.first.get<0>() - endPoint.get<0>());
            auto ydiff2 = std::abs(pointPair2.first.get<1>() - endPoint.get<1>());
            if(xdiff1 < 2 && ydiff1 < 2 && xdiff2 < 2 && ydiff2 < 2){
              if(bg::length(pointPair1.second) < bg::length(interPoints1[i].second)){
                proceed = false;
                break; 
              }else{
                proceed = false;
                keepTrackL[count] = std::make_pair(std::make_pair(p1, interPoints1[i].second), std::make_pair(endPoint, lsTemp)); 
                lineStrings3d[count] = convertLineStringToLineString3d(ls);
                break;
              }
            }
            count++;
          }
          if(proceed){
            std::vector<double>xs;std::vector<double>ys;
            xs.push_back(p1.get<0>());xs.push_back(endPoint.get<0>());
            ys.push_back(p1.get<1>());ys.push_back(endPoint.get<1>());
            auto lineDetails = linearRegression(xs, ys);
            auto slopeDiff = std::abs(std::get<0>(lineDetails)-std::get<0>(odomLineDetails));
            if(slopeDiff < 0.1 && lanelet::geometry::length(lanelet::utils::toHybrid(convertLineStringToLineString3d(ls))) < 27){
              keepTrackL.push_back(std::make_pair(std::make_pair(p1, interPoints1[i].second), std::make_pair(endPoint, lsTemp)));
              lineStrings3d.push_back(convertLineStringToLineString3d(ls));
            }
          }
        }
      }// Bool check closes
    }
  }//For loop of i closes  

   
  //if the second end point only exist, then below logic capture that and
  //construct the linestring
  std::vector<std::pair<std::pair<Point, LineString>, std::pair<Point, LineString>>> keepTrack;
  for(size_t i=0; i<interPoints2.size(); i++){
    auto p1 = interPoints2[i].first;
    double smallA = 1000.;
    size_t selectedIndex = 0;
    double selectedAngle = 1000.;
    for(size_t j=0; j<interPoints1.size(); j++){
      auto p2 = interPoints1[j].first;
      double angle = atan2(p2.get<1>() - p1.get<1>(), p2.get<0>() - p1.get<0>()) * 180 / 3.14159265; 
      if(std::abs(angle) < smallA){
        //Finding the line points with smallest angle 
        smallA = std::abs(angle);
        selectedIndex = j;
        selectedAngle = std::abs(angle);
      }
    }//For loop of j closes
    if(std::abs(selectedAngle) < 1){
      //There are both end points and that is already captured
    }else{
      //There is no first end point
     // In some case you have only one point, so it would be hard to construct a linestring. The solution is is to find the end point of that line, then get the slope equation form of that line to find out the end point upto the x of the odom point..
      Point secondPoint = interPoints2[i].second.back();
      Point endPoint = findInterpolatedPointGivenX(p1.get<0>(), p1.get<1>(), secondPoint.get<0>(), secondPoint.get<1>(), oPoint1.get<0>());
      LineString ls;ls.push_back(endPoint);ls.push_back(p1);
      if(!checkLineStrings3d(lineStrings3d, ls) && !std::isnan(endPoint.get<1>())){
        LineString lsTemp;
        if(keepTrack.size() == 0){
          std::vector<double>xs;std::vector<double>ys;
          xs.push_back(endPoint.get<0>());xs.push_back(p1.get<0>());
          ys.push_back(endPoint.get<1>());ys.push_back(p1.get<1>());
          auto lineDetails = linearRegression(xs, ys);
          auto slopeDiff = std::abs(std::get<0>(lineDetails)-std::get<0>(odomLineDetails));
          if(slopeDiff < 0.1 && lanelet::geometry::length(lanelet::utils::toHybrid(convertLineStringToLineString3d(ls))) < 27){
            keepTrack.push_back(std::make_pair(std::make_pair(endPoint, lsTemp), std::make_pair(p1, interPoints2[i].second)));
            //lineStrings.push_back(ls);
            lineStrings3d.push_back(convertLineStringToLineString3d(ls));
          }
        }else{
          size_t count = 0 ;
          bool proceed = true;
          for(auto pair: keepTrack){
            auto pointPair1 = pair.first; auto pointPair2 = pair.second;
            auto xdiff1 = std::abs(pointPair1.first.get<0>() - endPoint.get<0>());
            auto ydiff1 = std::abs(pointPair1.first.get<1>() - endPoint.get<1>());
            auto xdiff2 = std::abs(pointPair2.first.get<0>() - p1.get<0>());
            auto ydiff2 = std::abs(pointPair2.first.get<1>() - p1.get<1>());
            if(xdiff1 < 2 && ydiff1 < 2 && xdiff2 < 2 && ydiff2 < 2){
              if(bg::length(pointPair2.second) > bg::length(interPoints2[i].second)){
                proceed = false;
                break; 
              }else{
                proceed = false;
                keepTrack[count] = std::make_pair(std::make_pair(endPoint, lsTemp), std::make_pair(p1, interPoints2[i].second)); 
                //lineStrings[count] = ls;
                lineStrings3d[count] = convertLineStringToLineString3d(ls);
                break;
              }
            }
            count++;
          }
          if(proceed){
            std::vector<double>xs;std::vector<double>ys;
            xs.push_back(endPoint.get<0>());xs.push_back(p1.get<0>());
            ys.push_back(endPoint.get<1>());ys.push_back(p1.get<1>());
            auto lineDetails = linearRegression(xs, ys);
            auto slopeDiff = std::abs(std::get<0>(lineDetails)-std::get<0>(odomLineDetails));
            if(slopeDiff < 0.1 && lanelet::geometry::length(lanelet::utils::toHybrid(convertLineStringToLineString3d(ls))) < 27){
              keepTrack.push_back(std::make_pair(std::make_pair(endPoint, lsTemp), std::make_pair(p1, interPoints2[i].second)));
              //lineStrings.push_back(ls);
              lineStrings3d.push_back(convertLineStringToLineString3d(ls));
            }
          }
        }
      }//Bool check closes
    }
  }//For loop of i closes 
   

  return lineStrings3d;
}

void removeInterPointDuplicates(std::vector<std::pair<Point, LineString>>& interPoints1){
  auto tempInterPoints1 = interPoints1;
  interPoints1.clear();
  std::vector<size_t> eraseVec;
  for(size_t j=0; j<tempInterPoints1.size(); j++){
    auto interPointPair1 = tempInterPoints1[j];
    auto point1 = interPointPair1.first; 
    auto line1 = interPointPair1.second; 
    for(size_t k=0; k<tempInterPoints1.size(); k++){
      if(j == k)
        continue;
      auto interPointPair2 = tempInterPoints1[k];
      auto point2 = interPointPair2.first; 
      auto line2 = interPointPair2.second; 
      auto xdiff = std::abs(point2.get<0>()- point1.get<0>());
      auto ydiff = std::abs(point2.get<1>()- point1.get<1>());
      if(xdiff < 0.1 && ydiff < 0.1){
        auto selected = j;
        if(bg::length(line1) > bg::length(line2))
          selected  = k;
        eraseVec.push_back(selected);
      }
    }
  }

  for(size_t j=0; j<tempInterPoints1.size(); j++){
    if(std::find(eraseVec.begin(), eraseVec.end(), j) != eraseVec.end())
      continue;
    else{
      interPoints1.push_back(tempInterPoints1[j]);
    }
  }
  eraseVec.clear();
  tempInterPoints1.clear();
}




bool isLeftPoint(lanelet::Point3d a, lanelet::Point3d b, lanelet::Point3d c){
  auto d = ((c.x() - a.x())*(b.y() - a.y())) - ((c.y() - a.y())*(b.x() - a.x()));

  return d > 0;
}


std::vector<std::pair<size_t, lanelet::LineString3d>> orderTheLineString3d(std::vector<lanelet::LineString3d> lineStrings){

  std::vector<std::pair<size_t, lanelet::LineString3d>> orderVec; 
  for(size_t i=0; i<lineStrings.size(); i++){
    auto ls1 = lineStrings[i];
    auto x1 = ls1.front().x();auto y1 = ls1.front().y();auto x2 = ls1.back().x();auto y2 = ls1.back().y();
    auto x = (x1+x2)/2;auto y = (y1+y2)/2;
    size_t leftCounter = 0;
    for(size_t j=0; j<lineStrings.size(); j++){
      if(i == j)
        continue;
      auto ls2 = lineStrings[j];
      if(isLeftPoint(ls2.front(), ls2.back(), lanelet::Point3d(lanelet::utils::getId(), x, y, 0)))
      {
        leftCounter++;
      }
    }
    orderVec.push_back(std::make_pair(leftCounter, ls1));
  }
  
  //Sort the vector
  std::sort(orderVec.begin(), orderVec.end(), [](const std::pair<size_t, lanelet::LineString3d>& a, const std::pair<size_t, lanelet::LineString3d>& b) {return a.first < b.first;});

  return orderVec;
}

void createLaneletsOnTheGo(std::vector<std::pair<size_t, lanelet::LineString3d>> orderVec, lanelet::Lanelets& lanelets){
  if(orderVec.size() > 1){
    //Construct the lanelets based on the order is the number order vec
    //is 2 then, there is only one lanelets and if it is three, then
    //there will be two lanelets and the list goes on
    for(size_t i=0; i<orderVec.size()-1; i++){
      //Creating lanelets
      auto left = orderVec[i].second;
      auto right = orderVec[i+1].second;
      if(left != right){
        lanelet::Lanelet lanelet(lanelet::utils::getId(), left, right);
        lanelets.push_back(lanelet);
      }
    }
  }
}

void addIntermediateLine(std::vector<std::pair<size_t, lanelet::LineString3d>>& orderVec, std::vector<std::vector<std::pair<size_t, lanelet::LineString3d>>> lineStringVec){
  if(orderVec.size() > 1){
    auto tempOrderVec = orderVec;
    orderVec.clear();
    size_t order = 0;
    bool first = true;
    for(size_t i=0; i<tempOrderVec.size()-1; i++){
      auto ls3d1 = tempOrderVec[i].second;
      auto ls3d2 = tempOrderVec[i+1].second;
      auto x1 = ls3d1.front().x();auto y1 = ls3d1.front().y();auto x2 = ls3d1.back().x();auto y2 = ls3d1.back().y();
      auto x = (x1+x2)/2;auto y = (y1+y2)/2;
      auto tempMidP = lanelet::Point3d(lanelet::utils::getId(), x, y, 0);
      auto d = lanelet::geometry::distance(tempMidP, lanelet::utils::toHybrid(ls3d2));
      ROS_INFO_STREAM("d: "<<d);
      if(first)
        orderVec.push_back(std::make_pair(order,ls3d1));
      if(d > 5 && d < 7){
        double dist = (d/2)* -1;
        auto newLine2D = lanelet::geometry::offsetNoThrow(lanelet::utils::to2D(ls3d1), dist);
        auto tempP3 = lanelet::Point3d(lanelet::utils::getId(), newLine2D.front().x(), newLine2D.front().y(), 0);
        auto tempP4 = lanelet::Point3d(lanelet::utils::getId(), newLine2D.back().x(), newLine2D.back().y(), 0);
        order++;
        lanelet::LineString3d newLS(lanelet::utils::getId(), {tempP3, tempP4});
        orderVec.push_back(std::make_pair(order, newLS));
      }
      order++;
      orderVec.push_back(std::make_pair(order, ls3d2));
      first = false;
    }
  }
}

bool  scanNextFew(std::vector<std::vector<std::pair<size_t, lanelet::LineString3d>>> lineStringVec, size_t start, size_t end, int size){
  bool enable = false;
  if(end > lineStringVec.size())
    end = lineStringVec.size();
  for(size_t i=start; i<end; i++){
    auto orderVec = lineStringVec[i];
    if(orderVec.size() > size){
      enable = true;
      break; 
    }
  }
  
  return enable;
}

std::pair<bool, size_t> scanNextFewFirst(std::vector<std::vector<std::pair<size_t, lanelet::LineString3d>>> lineStringVec, size_t start, size_t end, int size){
  bool enable = false;
  int count = 0;
  size_t selectedIndex = 0;
  bool first = false;
  if(end > lineStringVec.size())
    end = lineStringVec.size();
  for(size_t i=start; i<end; i++){
    auto orderVec = lineStringVec[i];
    if(orderVec.size() > size){
      count++;
      if(!first){
        first = true;
        selectedIndex = i;
      }
    }
  }
  
  if(count > 0)
    enable = true;

  return std::make_pair(enable, selectedIndex);
}


bool scanNextLast(std::vector<std::vector<std::pair<size_t, lanelet::LineString3d>>> lineStringVec, size_t start, size_t end){
  bool enable = false;
  int count = 0;
  if(end > lineStringVec.size()){
    enable = true;
  }
  if(start > lineStringVec.size())
    enable = true;
  
  return enable;
}


double findAverageWidth(std::vector<std::pair<size_t, lanelet::LineString3d>> orderVec){
  double d = 0.;
  int count = 0;
  for(size_t i=0; i<orderVec.size()-1; i++){
    auto ls3d1 = orderVec[i].second;
    auto ls3d2 = orderVec[i+1].second;
    auto x1 = ls3d1.front().x();auto y1 = ls3d1.front().y();auto x2 = ls3d1.back().x();auto y2 = ls3d1.back().y();
    auto x = (x1+x2)/2;auto y = (y1+y2)/2;
    lanelet::Point3d point3d{lanelet::utils::getId(), x, y, 0};
    d += lanelet::geometry::distance(point3d, lanelet::utils::toHybrid(ls3d2));
    count++;
  }

  return std::abs((d/count));
}

std::vector<lanelet::LineString3d> removeTheClosestLine(std::vector<std::pair<size_t, lanelet::LineString3d>> orderVec, double threshold){
  std::vector<lanelet::LineString3d> returnVec;
  bool first = true;
  bool skip = false;
  for(size_t i=0; i<orderVec.size()-1; i++){
    auto ls3d1 = orderVec[i].second;
    auto ls3d2 = orderVec[i+1].second;
    auto x1 = ls3d1.front().x();auto y1 = ls3d1.front().y();auto x2 = ls3d1.back().x();auto y2 = ls3d1.back().y();
    auto x = (x1+x2)/2;auto y = (y1+y2)/2;
    lanelet::Point3d point3d{lanelet::utils::getId(), x, y, 0};
    auto d = lanelet::geometry::distance(point3d, lanelet::utils::toHybrid(ls3d2));
    
    if(d < threshold){
      skip = true;
    }else
      skip = false;
    
    if(!skip || first){
      returnVec.push_back(ls3d1);
      first = false;
    }

  }
  
  if(!skip){
    auto ls3d1 = orderVec[orderVec.size()-1].second;
    returnVec.push_back(ls3d1);
  }

  return returnVec;

}

std::vector<lanelet::LineString3d> removeTheClosestAndSmallLine(std::vector<std::pair<size_t, lanelet::LineString3d>> orderVec, double threshold){
  std::vector<lanelet::LineString3d> returnVec;
  bool first = true;
  bool skip = false;
  std::vector<size_t> allowed;
  std::vector<size_t> rejected;
  for(size_t i=0; i<orderVec.size()-1; i++){
    if(std::find(rejected.begin(), rejected.end(), i) != rejected.end())
      continue;
    auto ls3d1 = orderVec[i].second;
    auto ls3d2 = orderVec[i+1].second;
    auto x1 = ls3d1.front().x();auto y1 = ls3d1.front().y();auto x2 = ls3d1.back().x();auto y2 = ls3d1.back().y();
    auto x = (x1+x2)/2;auto y = (y1+y2)/2;
    lanelet::Point3d point3d{lanelet::utils::getId(), x, y, 0};
    auto d = lanelet::geometry::distance(point3d, lanelet::utils::toHybrid(ls3d2));

    if(d < threshold){
      skip = true;
    }else
      skip = false;

    if(!skip){
      if(std::find(allowed.begin(), allowed.end(), i) != allowed.end())
        continue;
      allowed.push_back(i);
    }else{
      lanelet::LineString3d ls3d; 
      auto length1 = lanelet::geometry::length(lanelet::utils::toHybrid(ls3d1));
      auto length2 = lanelet::geometry::length(lanelet::utils::toHybrid(ls3d2));
      if(length1 < length2){
        if(std::find(allowed.begin(), allowed.end(), i+1) != allowed.end())
          continue;
        allowed.push_back(i+1);
      }else{
        if(std::find(allowed.begin(), allowed.end(), i) != allowed.end())
          continue; 
        allowed.push_back(i);
        rejected.push_back(i+1);
        skip = false;
      }
    }

  }
  
  if(!skip){
    allowed.push_back(orderVec.size()-1);
  }
  ROS_INFO_STREAM("size: "<<allowed.size());
  for(size_t i=0; i<orderVec.size(); i++){
    if(std::find(allowed.begin(), allowed.end(), i) != allowed.end()){
      auto ls3d1 = orderVec[i].second;
      returnVec.push_back(ls3d1); 
    }
  }
  return returnVec;

}


void removeAndReplaceSmallerLS(std::vector<std::pair<size_t, lanelet::LineString3d>>& orderVec){

  auto tempVec = orderVec;
  auto avgDist = findAverageWidth(orderVec);
  for(size_t i=0; i<tempVec.size(); i++){
    auto ls3d = tempVec[i].second;
    auto length = lanelet::geometry::length(lanelet::utils::toHybrid(ls3d));
    if(length < 22){
      if((i+1) <= tempVec.size()-1 && lanelet::geometry::length(lanelet::utils::toHybrid(tempVec[i+1].second)) > 22){
        auto newLine2D1 = lanelet::geometry::offsetNoThrow(lanelet::utils::to2D(tempVec[i+1].second), avgDist);
        auto tempP3 = lanelet::Point3d(lanelet::utils::getId(), newLine2D1.front().x(), newLine2D1.front().y(), 0);
        auto tempP4 = lanelet::Point3d(lanelet::utils::getId(), newLine2D1.back().x(), newLine2D1.back().y(), 0);
        lanelet::LineString3d newLS(lanelet::utils::getId(), {tempP3, tempP4});
        orderVec[i] = std::make_pair(tempVec[i].first, newLS);
      }else if(i == tempVec.size()-1 && lanelet::geometry::length(lanelet::utils::toHybrid(tempVec[i-1].second)) > 22){
        avgDist *= -1;
        auto newLine2D1 = lanelet::geometry::offsetNoThrow(lanelet::utils::to2D(tempVec[i-1].second), avgDist);
        auto tempP3 = lanelet::Point3d(lanelet::utils::getId(), newLine2D1.front().x(), newLine2D1.front().y(), 0);
        auto tempP4 = lanelet::Point3d(lanelet::utils::getId(), newLine2D1.back().x(), newLine2D1.back().y(), 0);
        lanelet::LineString3d newLS(lanelet::utils::getId(), {tempP3, tempP4});
        orderVec[i] = std::make_pair(tempVec[i].first, newLS);
      }
    }
  }
}

void addArbitaryLine(std::vector<std::vector<std::pair<size_t, lanelet::LineString3d>>>& lineStringVec){
  auto tempVec = lineStringVec;
  lineStringVec.clear();
  for(size_t i=0; i<tempVec.size(); i++){
    ROS_INFO_STREAM("All i: "<<i);
    auto currentVec = tempVec[i];
    bool proceed = false;
    if(i==0){
      auto result = scanNextFewFirst(tempVec, i+1, i+15, currentVec.size());
      if(result.first){
        ROS_INFO_STREAM("index: "<<result.second);
        auto index = result.second;
        auto nextVec = tempVec[index]; 
        std::vector<size_t> brokenLines;
        for(size_t j=0;j<nextVec.size(); j++){
          auto ls3d1 = nextVec[j].second;
          auto p3d1 = ls3d1.front();
          auto lineDeatils1 = applyLinearRegressionOnPoint(Point(p3d1.x(), p3d1.y()), Point(ls3d1.back().x(), ls3d1.back().y()));
          bool found = false;
          bool smallD = 1000.;
          size_t selectedIndex = 0;
          for(size_t k=0; k<currentVec.size(); k++){
            auto ls3d2 = currentVec[k].second; 
            auto p3d2 = ls3d2.back();
            auto y = std::get<0>(lineDeatils1)*p3d2.x()+std::get<1>(lineDeatils1);
            auto y_diff = std::abs(p3d2.y() - y);
            ROS_INFO_STREAM("y_diff0: "<<y_diff);
            if(y_diff < 1.6){//1.2 works or 0.2 works
              found = true;
              break;
            }
          }
          if(!found){
            brokenLines.push_back(j);
          }
        }

        if(brokenLines.size() > 0){
          std::vector<lanelet::LineString3d> lineStrings;
          for(auto& pair: currentVec){
            lineStrings.push_back(pair.second);
          }

          for(size_t j=0; j<brokenLines.size(); j++){
            if(nextVec.size() == currentVec.size())
             break; 
            auto avgDist = findAverageWidth(nextVec);
            auto ls3d1 = nextVec[brokenLines[j]].second;
            auto p3d1 = ls3d1.front();
            auto lineDeatils1 = applyLinearRegressionOnPoint(Point(p3d1.x(), p3d1.y()), Point(ls3d1.back().x(), ls3d1.back().y()));
            double smallD = 1000.;
            double selectedIndex = 0;
            double finalD = 0.;
            for(size_t k=0; k<currentVec.size(); k++){
              auto ls3d2 = currentVec[k].second; 
              auto p3d2 = ls3d2.back();
              auto y = std::get<0>(lineDeatils1)*p3d2.x()+std::get<1>(lineDeatils1);
              auto y_diff = std::abs(p3d2.y() - y);
              if(y_diff < smallD){
                smallD = y_diff;
                selectedIndex = k;
                finalD = y_diff;
              }
            }
            auto ls3d = currentVec[selectedIndex].second;
            auto newLine2D1 = lanelet::geometry::offsetNoThrow(lanelet::utils::to2D(ls3d), avgDist);
            auto y1 = std::get<0>(lineDeatils1)*newLine2D1.back().x()+std::get<1>(lineDeatils1);
            auto y_diff1 = std::abs(newLine2D1.back().y() - y1);
            
            avgDist *= -1;
            auto newLine2D2 = lanelet::geometry::offsetNoThrow(lanelet::utils::to2D(ls3d), avgDist);
            auto y2 = std::get<0>(lineDeatils1)*newLine2D2.back().x()+std::get<1>(lineDeatils1);
            auto y_diff2 = std::abs(newLine2D2.back().y() - y2);
            lanelet::Point3d tempP3;lanelet::Point3d tempP4;
            if(y_diff1 < y_diff2){
              tempP3 = lanelet::Point3d(lanelet::utils::getId(), newLine2D1.front().x(), newLine2D1.front().y(), 0);
              tempP4 = lanelet::Point3d(lanelet::utils::getId(), newLine2D1.back().x(), newLine2D1.back().y(), 0);
            }else{
              tempP3 = lanelet::Point3d(lanelet::utils::getId(), newLine2D2.front().x(), newLine2D2.front().y(), 0);
              tempP4 = lanelet::Point3d(lanelet::utils::getId(), newLine2D2.back().x(), newLine2D2.back().y(), 0);
            } 
            lanelet::LineString3d newLS(lanelet::utils::getId(), {tempP3, tempP4});
            lineStrings.push_back(newLS);
            auto orderVec = orderTheLineString3d(lineStrings);
            currentVec = orderVec;
          }
          auto orderVec = orderTheLineString3d(lineStrings);
          lineStrings = removeTheClosestLine(orderVec, 1.);
          orderVec = orderTheLineString3d(lineStrings);
          lineStringVec.push_back(orderVec);
          tempVec[i] = orderVec;
        }else{
          lineStringVec.push_back(currentVec);
        }
      }else
        lineStringVec.push_back(currentVec);
    }else if(scanNextFew(tempVec, i+1, i+15, currentVec.size()) && tempVec[i-1].size() > currentVec.size()){
      proceed = true;
    }else if(scanNextLast(tempVec, i+1, i+15) && tempVec[i-1].size() > currentVec.size()){
      proceed = true;
      ROS_INFO_STREAM("Last i: "<<i);
    }else{
      removeAndReplaceSmallerLS(currentVec);
      lineStringVec.push_back(currentVec);
    }
    if(proceed && currentVec.size() > 0){
      auto prevVec = tempVec[i-1]; 
      ROS_INFO_STREAM("-------- i: "<<i<<" current vec size: "<<currentVec.size()<<" prevVec size "<<prevVec.size());
      std::vector<size_t> brokenLines;
      for(size_t j=0;j<prevVec.size(); j++){
        auto ls3d1 = prevVec[j].second;
        auto p3d1 = ls3d1.back();
        auto lineDeatils1 = applyLinearRegressionOnPoint(Point(p3d1.x(), p3d1.y()), Point(ls3d1.front().x(), ls3d1.front().y()));
        bool found = false;
        bool smallD = 1000.;
        size_t selectedIndex = 0;
        for(size_t k=0; k<currentVec.size(); k++){
          auto ls3d2 = currentVec[k].second; 
          auto p3d2 = ls3d2.front();
          auto y = std::get<0>(lineDeatils1)*p3d2.x()+std::get<1>(lineDeatils1);
          auto y_diff = std::abs(p3d2.y() - y);
          auto d = lanelet::geometry::distance(p3d1, p3d2);
          if(d < 1. || y_diff < 0.2){
            found = true;
            break;
          }
        }
        if(!found){
          brokenLines.push_back(j);
        }
      }
      
      //If there is broken lines, then we need to create the line that extends
      //from previous line
      if(brokenLines.size() > 0){
        std::vector<lanelet::LineString3d> lineStrings;
        for(auto& pair: currentVec){
          lineStrings.push_back(pair.second);
        }

        for(size_t j=0; j<brokenLines.size(); j++){
          if(prevVec.size() == currentVec.size())
           break; 
          ROS_INFO_STREAM("Broken line: "<< brokenLines[j]<<" currentVec size:: "<<currentVec.size());
          auto avgDist = findAverageWidth(prevVec);
          /*if(currentVec.size() > 1){
            double dist; 
            int selectedLine;
            if(brokenLines[j] > 0){
              selectedLine = brokenLines[j] - 1;
              dist = avgDist*-1;
            }else{
              selectedLine = 1;
              dist = avgDist;
            }
            auto ls3d1 = currentVec[selectedLine].second;
            auto newLine2D = lanelet::geometry::offsetNoThrow(lanelet::utils::to2D(ls3d1), dist);
            auto tempP3 = lanelet::Point3d(lanelet::utils::getId(), newLine2D.front().x(), newLine2D.front().y(), 0);
            auto tempP4 = lanelet::Point3d(lanelet::utils::getId(), newLine2D.back().x(), newLine2D.back().y(), 0);
            lanelet::LineString3d newLS(lanelet::utils::getId(), {tempP3, tempP4});
            lineStrings.push_back(newLS);
            auto orderVec = orderTheLineString3d(lineStrings);
            currentVec = orderVec;
          }else{ */
          if(true){
            auto ls3d1 = prevVec[brokenLines[j]].second;
            auto p3d1 = ls3d1.back();
            double smallD = 1000.;
            double selectedIndex = 0;
            double finalD = 0.;
            for(size_t k=0; k<currentVec.size(); k++){
              auto ls3d2 = currentVec[k].second; 
              auto pProj = lanelet::geometry::project(ls3d2, p3d1);
              lanelet::Point3d p3d2{lanelet::utils::getId(), pProj.x(), pProj.y(), 0};
              auto d = lanelet::geometry::distance(p3d1, p3d2);
              //ROS_INFO_STREAM(p3d1.x()<<" "<<p3d1.y()<<" , "<<p3d2.x()<<" "<<p3d2.y()<<" d: "<<d);
              if(d < smallD){
                smallD = d;
                selectedIndex = k;
                finalD = d;
              }
            }
            auto ls3d = currentVec[selectedIndex].second;
            auto newLine2D1 = lanelet::geometry::offsetNoThrow(lanelet::utils::to2D(ls3d), avgDist);
            auto x1 = newLine2D1.front().x();auto y1 = newLine2D1.front().y();auto x2 = newLine2D1.back().x();auto y2 = newLine2D1.back().y();
            auto x = (x1+x2)/2; auto y = (y1+y2)/2;
            
            //Construct a lane to check the line angle
            double angle1 = std::abs(atan2(y - p3d1.y(), x - p3d1.x()) * 180 / 3.14159265);
            avgDist *= -1;
            auto newLine2D2 = lanelet::geometry::offsetNoThrow(lanelet::utils::to2D(ls3d), avgDist);
            x1 = newLine2D2.front().x();y1 = newLine2D2.front().y();x2 = newLine2D2.back().x();y2 = newLine2D2.back().y();
            x = (x1+x2)/2;y = (y1+y2)/2;
            
            //Construct a lane to check the line angle
            double angle2 = std::abs(atan2(y - p3d1.y(), x - p3d1.x()) * 180 / 3.14159265);
            lanelet::Point3d tempP3;lanelet::Point3d tempP4;
            if(angle1 < angle2){
              tempP3 = lanelet::Point3d(lanelet::utils::getId(), newLine2D1.front().x(), newLine2D1.front().y(), 0);
              tempP4 = lanelet::Point3d(lanelet::utils::getId(), newLine2D1.back().x(), newLine2D1.back().y(), 0);
            }else{
              tempP3 = lanelet::Point3d(lanelet::utils::getId(), newLine2D2.front().x(), newLine2D2.front().y(), 0);
              tempP4 = lanelet::Point3d(lanelet::utils::getId(), newLine2D2.back().x(), newLine2D2.back().y(), 0);
            } 
            lanelet::LineString3d newLS(lanelet::utils::getId(), {tempP3, tempP4});
            lineStrings.push_back(newLS);
            auto orderVec = orderTheLineString3d(lineStrings);
            currentVec = orderVec;
          }
        }
                
                
        auto orderVec = orderTheLineString3d(lineStrings);
        lineStrings = removeTheClosestLine(orderVec, 1.);
        orderVec = orderTheLineString3d(lineStrings);
        lineStringVec.push_back(orderVec);
        tempVec[i] = orderVec;

      }else{
        removeAndReplaceSmallerLS(currentVec);
        lineStringVec.push_back(currentVec);
      }
    }
  } 
}

std::vector<lanelet::LineString3d> reduceNumberOfLineStrings(std::vector<std::vector<std::pair<size_t, lanelet::LineString3d>>>& lineStringVec){
  std::vector<lanelet::LineString3d> activeLineString;
  for(size_t i= 0; i<lineStringVec.size(); i++){
    if(i == 0){
      auto orderVec = lineStringVec[i];
      for(size_t j=0; j<orderVec.size(); j++){
        activeLineString.push_back(orderVec[j].second);
      }        
    }else{
      auto orderVec = lineStringVec[i];
      std::vector<size_t> foundVec;
      for(size_t j=0; j<activeLineString.size(); j++){
        auto ls3d1 = activeLineString[j];
        auto p3d1 = ls3d1.back();
        double smallD = 1000.;
        size_t selectedIndex = 0;
        bool found = false;
        for(size_t k=0; k<orderVec.size(); k++){
          auto ls3d2 = orderVec[k].second; 
          auto p3d2 = ls3d2.front();
          auto d = lanelet::geometry::distance(p3d1, p3d2);
          if(d < 1.5 && d < smallD){
            found = true;
            selectedIndex = k;
            smallD = d;
          }
        }

        if(found){
          /*
            *-----------*p3d1 p3d2*----------------*p3d3
          */
          auto p3d2 = orderVec[selectedIndex].second.front();
          auto x1 = p3d1.x();
          auto x2 = p3d2.x();
          auto y1 = p3d1.y();
          auto y2 = p3d2.y();
          auto x = (x1+x2)/2;auto y = (y1+y2)/2;
          ls3d1[ls3d1.size()-1] = p3d2;
          //ls3d1.push_back(lanelet::Point3d(lanelet::utils::getId(), x, y, 0)); 
          ls3d1.push_back(orderVec[selectedIndex].second.back()); 
          /* 
          for(auto p3d: orderVec[selectedIndex].second){
            ls3d1.push_back(p3d); 
          }*/
          foundVec.push_back(selectedIndex);
          activeLineString[j] = ls3d1;
        }
      }
      for(size_t k=0; k<orderVec.size(); k++){
        if(std::find(foundVec.begin(), foundVec.end(), k) != foundVec.end())
          continue;
        else{
          activeLineString.push_back(orderVec[k].second);
        }
      }
    } 
  }
  
  return activeLineString;
}

void createLanelets(std::vector<std::vector<std::pair<size_t, lanelet::LineString3d>>> lineStringVec, lanelet::Lanelets& lanelets){

  for(auto orderVec: lineStringVec){
    if(orderVec.size() > 1){
      //Construct the lanelets based on the order is the number order vec
      //is 2 then, there is only one lanelets and if it is three, then
      //there will be two lanelets and the list goes on
      for(size_t i=0; i<orderVec.size()-1; i++){
        //Creating lanelets
        auto left = orderVec[i].second;
        auto right = orderVec[i+1].second;
        if(left != right){
          lanelet::Lanelet lanelet(lanelet::utils::getId(), left, right);
          lanelets.push_back(lanelet);
        }
      }
    }
  }
}

/*
 * Get File Name from a Path with or without extension
 */
std::string getFileName(std::string filePath, bool withExtension = true)
{
    // Create a Path object from File Path
    filesys::path pathObj(filePath);
    // Check if file name is required without extension
    if(withExtension == false)
    {
        // Check if file has stem i.e. filename without extension
        if(pathObj.has_stem())
        {
            // return the stem (file name without extension) from path object
            return pathObj.stem().string();
        }
        return "";
    }
    else
    {
        // return the file name with extension from path object
        return pathObj.filename().string();
    }
}

void laneletToOpenDriveConverter(std::vector<std::vector<std::pair<size_t, lanelet::LineString3d>>> lineStringVec, std::string laneletToOD_file, std::string bag_file){
  int count = 0;
  lanelet::LineString3d refLS;
  std::vector<json> data;
  double prevM = 0;
  lanelet::Point3d prevPoint;
  double sSum = 0;
  int noOfLanesStart;
  for(auto orderVec: lineStringVec){
    json jData; 
    jData["laneWidth"] = findAverageWidth(orderVec);
    jData["noLanes"] = orderVec.size()-1;
    lanelet::LineString3d refLS;
    if(orderVec.size()%2 == 0){
      int rem = orderVec.size()/2;
      rem -= 1;
      auto firstLS = orderVec[rem].second;
      auto secondLS = orderVec[rem+1].second;
      lanelet::Lanelet LL(lanelet::utils::getId(), firstLS, secondLS);
      auto centerLine = LL.centerline();
      lanelet::Point3d p1{lanelet::utils::getId(), centerLine.front().x(), centerLine.front().y(), 0};
      lanelet::Point3d p2{lanelet::utils::getId(), centerLine.back().x(), centerLine.back().y(), 0};
      lanelet::LineString3d tempLS(lanelet::utils::getId(),{p1, p2});
      refLS = tempLS;
    }else{
      int rem = orderVec.size()/2;
      refLS = orderVec[rem].second;
    }
    jData["length"] = lanelet::geometry::length(lanelet::utils::toHybrid(refLS));
    double mDiff = 0.;
    auto lineDetails = applyLinearRegressionOnLS(refLS, "front");
    if(count == 0){
      noOfLanesStart = orderVec.size()-1;
      jData["mDiff"] = 0.;
      jData["sStart"] = 0.;
      sSum = 0.;
    }else{
      sSum += lanelet::geometry::distance(prevPoint, refLS.front());
      jData["sStart"] = sSum;
      mDiff = prevM - std::get<0>(lineDetails);
      jData["mDiff"] = mDiff*0.05*-1;
    }
    prevM = std::get<0>(lineDetails);
    prevPoint = refLS.front();
    count++;
    data.push_back(jData);
  } 
  
  ROS_INFO_STREAM("LaneletToOpenDrive data size: "<<data.size());
  
  //Save the data
  json dataJ1(data);
  json finalJson = {
    {"noOfLanesStart", noOfLanesStart},
    {"data", dataJ1},
    {"file", bag_file}
  };
  auto filename = getFileName(bag_file, false);
  std::ofstream o1(laneletToOD_file+filename+".laneletToOD");
  o1 << std::setw(4) << finalJson << std::endl;
}


void createLaneletsMap(std::vector<LineString> allLineStrings, std::vector<std::pair<double,double>> vehicle_odom_double, std::string laneletsmap_file, std::string laneletToOD_file, std::string bag_file){
  lanelet::Lanelets lanelets;
  std::vector<std::vector<std::pair<size_t, lanelet::LineString3d>>> lineStringVec;

  //Loop through ODOM range
  bool init = false;
  Point p1; Point p2;
  std::vector<double> xs;std::vector<double> ys;
  double odomLineLength = 25.;
  double lineLength = 10.;
  std::vector<std::tuple<size_t, lanelet::Point3d, lanelet::Point3d>> prevLines;
  for(size_t i=0; i<vehicle_odom_double.size()-1; i++){
    if(!init){
      xs.push_back(vehicle_odom_double[i].first);
      ys.push_back(vehicle_odom_double[i].second);
      p1.set<0>(vehicle_odom_double[i].first);
      p1.set<1>(vehicle_odom_double[i].second);
      init = true;
    }else{
      xs.push_back(vehicle_odom_double[i].first);
      ys.push_back(vehicle_odom_double[i].second);
      p2.set<0>(vehicle_odom_double[i].first);
      p2.set<1>(vehicle_odom_double[i].second);
      LineString odomLS;
      odomLS.push_back(p1);
      odomLS.push_back(p2);
      if(bg::length(odomLS) > odomLineLength){
        ROS_INFO_STREAM("--------"<<p1.get<0>()<<" "<<p1.get<1>()<<" , "<<p2.get<0>()<<" "<<p2.get<1>()<<" "<<i<<"/"<<vehicle_odom_double.size());
        std::vector<std::tuple<size_t, std::pair<lanelet::Point3d, lanelet::Point3d>, lanelet::LineString3d>> selectedLines;
      
        //We are considering slope of the normal line/perpendicular line to
        //the odom line
        //auto odomLineDetails = linearRegression(xs, ys);
        //auto perpLineDetails = getPerpendicularLineDetails(odomLineDetails, p1); 
        //auto perpLine = getLineFromDetails(perpLineDetails, p1);
        auto perpLine = getLineFromDetails(p1, p2, "first");
        std::vector<std::pair<Point, LineString>> interPoints1;
        //Loop through the lines for the first odom point
        for(size_t j=0; j < allLineStrings.size(); j++){
          auto line = allLineStrings[j];
          if(bg::length(line) < lineLength)
            continue;

          MultiPoint interPoint;
          bg::intersection(perpLine.second, line, interPoint);
          if(interPoint.size() > 0){
            auto pProj = lanelet::geometry::project(convertLineStringToLineString3d(line), pointToPoint3d(p1));
            interPoints1.push_back(std::make_pair(Point(pProj.x(), pProj.y()), line));
          }
        }
        //Remove the duplicates
        removeInterPointDuplicates(interPoints1);
        //perpLineDetails = getPerpendicularLineDetails(odomLineDetails, p2); 
        //perpLine = getLineFromDetails(perpLineDetails, p2);
        perpLine = getLineFromDetails(p1, p2, "second");
        std::vector<std::pair<Point,LineString>> interPoints2;
        //Loop through the lines for the second odom point
        for(size_t j=0; j < allLineStrings.size(); j++){
          auto line = allLineStrings[j];
          if(bg::length(line) < lineLength)
            continue; 
          
          MultiPoint interPoint;
          bg::intersection(perpLine.second, line, interPoint);
          if(interPoint.size() > 0){
            auto pProj = lanelet::geometry::project(convertLineStringToLineString3d(line), pointToPoint3d(p2));
            interPoints2.push_back(std::make_pair(Point(pProj.x(), pProj.y()), line));
          }
        }
        
        //Remove the duplicates
        removeInterPointDuplicates(interPoints2);
        
        //If we have two point with lowest angle that can be considered as
        //endpoints of one line string. In some case you have only one point,
        //then end point of that line will be the one end point of the
        //linestring. If there is no points, then, we have to do something
        //like, if it is near odom lines, construct the line etc.
        auto lineStrings3d = createLineStringsFromInterPoints(odomLS, interPoints1, interPoints2, p1, p2);        
        //Order the lines
        std::vector<std::pair<size_t, lanelet::LineString3d>> orderVec = orderTheLineString3d(lineStrings3d);

        //If distance is greater than 5, then add an intermediate line
        addIntermediateLine(orderVec, lineStringVec);

        for(auto& pair: orderVec){
          ROS_INFO_STREAM(lanelet::geometry::length(lanelet::utils::toHybrid(pair.second)));
        }

        if(orderVec.size() > 1){
          auto lineStrings = removeTheClosestLine(orderVec, 2.);
          orderVec = orderTheLineString3d(lineStrings);
        }
        lineStringVec.push_back(orderVec);
        
        //Create lanelets from linestrings
        //createLaneletsOnTheGo(orderVec, lanelets);

        xs.clear();
        ys.clear();
        init = false;

      }// if loop closes
    }// else close
  }//for loop i closes
  
  addArbitaryLine(lineStringVec);
  auto tempVec = lineStringVec;
  lineStringVec.clear(); 
  for(size_t i=0; i<tempVec.size(); i++){
    auto orderVec = tempVec[i];
    auto lineStrings = removeTheClosestAndSmallLine(orderVec, 1.);
    orderVec = orderTheLineString3d(lineStrings);
    lineStringVec.push_back(orderVec);
  }
  
  addArbitaryLine(lineStringVec);

  /*----Start of modification for opendrive to make one lanelet for whole road*/
  laneletToOpenDriveConverter(lineStringVec, laneletToOD_file, bag_file);
  auto lineStrings = reduceNumberOfLineStrings(lineStringVec);
  auto orderVec = orderTheLineString3d(lineStrings);
  lineStringVec.clear();
  lineStringVec.push_back(orderVec);
  createLanelets(lineStringVec, lanelets);
  /*----End of modification for opendrive to make one lanelet for whole road*/
  
  //Creating a laneletmap
  lanelet::LaneletMapUPtr laneletsMap = lanelet::utils::createMap(lanelets);
  //lanelet::projection::UtmProjector projector(lanelet::Origin({-33.8820738678, 151.197494122, 17.8450012207}));
  lanelet::projection::UtmProjector projector(lanelet::Origin({0,0}));
  lanelet::write(laneletsmap_file, *laneletsMap, projector);
  ROS_INFO_STREAM("Lanelet map is created: "<< laneletsMap->laneletLayer.exists(lanelets.front().id()));  
}

#endif //APPLICATIONS_HELPER_FUNCTIONS_HPP
