#include "behavior_tree_pipeline.hpp"

BehaviorTreePipeline::BehaviorTreePipeline()
{
    input_prompt.Initialise("/zio/odometry/rear");

    output_prompt.Initialise("/tock", pipes_out);
}



void
BehaviorTreePipeline::receive_message(const nav_msgs::Odometry::ConstPtr& input_odom) {

//  ROS_INFO_STREAM("sending tick to behavior tree" );


  ResetMessageFlags();

  input_prompt.PublishMessage(input_odom);

  if (WaitForMessages()) {
//    ROS_INFO_STREAM("received tock and return to read bag msgs");

//    if (publish_poles_corners) {
//      publish_poles_corners(output_poles.last_message, output_corners.last_message);
//    }
  }else {
    ROS_INFO_STREAM("No messages received and return to read bag msgs");
    output_prompt.last_message = NULL;
  }
}
