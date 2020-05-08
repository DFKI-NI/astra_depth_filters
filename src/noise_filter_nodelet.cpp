#include "astra_depth_filters/noise_filter_nodelet.h"
#include <pluginlib/class_list_macros.h>

void NoiseFilterNodelet::onInit()
{
  ROS_INFO("Initializing Noise Filter Nodelet");

  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh = getMTNodeHandle();
  ros::NodeHandle nh_priv = getMTPrivateNodeHandle();

  filter_.reset(new NoiseFilter(nh, nh_priv));
}

PLUGINLIB_EXPORT_CLASS(NoiseFilterNodelet, nodelet::Nodelet);
