//
// Created by stew on 3/06/20.
//

#ifndef TOP_DOWN_PROJECTION_DRAW_SHAPES_HPP
#define TOP_DOWN_PROJECTION_DRAW_SHAPES_HPP

class DrawItem
{
public:
  DrawItem() : alpha(255), shape_size(1) {}
  DrawItem(uint8_t _alpha, uint8_t _shape_size) : alpha(_alpha), shape_size(_shape_size) {}

  virtual void drawItem(cv::Mat &image, cv::Point &point, cv::Vec4b &colour) {

  }

  unsigned short alpha;
  unsigned short shape_size;
  std::string topic_name;
  std::string field_name = "intensity";
};


class CircleItem : public DrawItem
{
public:

  CircleItem() : DrawItem(255,3) {}
  CircleItem(uint8_t _alpha, uint8_t _shape_size) : DrawItem(_alpha, _shape_size) {}

  void drawItem(cv::Mat &image, cv::Point &point, cv::Vec4b &colour) {
    // Set the corresponding pixel to the RBG colour
    colour[3] = alpha;

//    std::cout << "here " << alpha << ", " << shape_size << ", " << topic_name << std::endl;
    // Set the alpha value for the blurred layer
    cv::circle(image, point, shape_size, colour, CV_FILLED);
  }
};

class PointItem : public DrawItem
{
public:

  PointItem() : DrawItem(255,3) {}
  PointItem(uint8_t _alpha, uint8_t _shape_size) : DrawItem(_alpha, _shape_size) {}

  void drawItem(cv::Mat &image, cv::Point &point, cv::Vec4b &colour) {
    // Set the corresponding pixel to the RBG colour
    colour[3] = alpha;

    // Set the alpha value for the blurred layer
    image.at<cv::Vec4b>(point) = colour;
  }
};




#endif //TOP_DOWN_PROJECTION_DRAW_SHAPES_HPP
