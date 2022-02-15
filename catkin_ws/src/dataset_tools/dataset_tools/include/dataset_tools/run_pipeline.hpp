#ifndef run_pipeline_h
#define run_pipeline_h

#include <ros/ros.h>

// include messages to write to bag file
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>

#include "pipeline_components.hpp"
#include "pipeline_images.hpp"


/*!
 * \brief Call an external pipeline (ros node) and wait for the responses
 *
 */

class RunPipeline {
public:
  RunPipeline();
  ~RunPipeline();

  bool WaitForMessages(float time_out = 1.);
  void ResetMessageFlags();

protected:

  std::vector<PipelineConnectorOutput*> pipes_out;
};



#endif
