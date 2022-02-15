#ifndef BEHAVIOR_TREE_PIPELINE_H
#define BEHAVIOR_TREE_PIPELINE_H
#include "run_pipeline.hpp"
#include <nav_msgs/Odometry.h>


class BehaviorTreePipeline : public RunPipeline {

public:
    BehaviorTreePipeline();
    ~BehaviorTreePipeline() {}

    void receive_message(const nav_msgs::Odometry::ConstPtr& input_odom);

    //no output function needed for this pipeline
//    std::function<void(const pcl::PointCloud<pcl::PointXYZIRC>::Ptr&, const pcl::PointCloud<pcl::PointXYZIRC>::Ptr&)> publish_poles_corners;

    PipelineOutput<nav_msgs::Odometry> output_prompt;

private:
    PipelineInput<nav_msgs::Odometry> input_prompt;


};

#endif // BEHAVIOR_TREE_PIPELINE_H
