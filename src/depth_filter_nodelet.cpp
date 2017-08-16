#include "astra_depth_filters/depth_filter_nodelet.h"
#include <pluginlib/class_list_macros.h>

void DepthFilterNodelet::onInit()
{
  ROS_INFO("Initializing Depth Filter Nodelet");

  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh = getMTNodeHandle();
  ros::NodeHandle nh_priv = getMTPrivateNodeHandle();

  filter_.reset(new DepthFilter(nh, nh_priv));
}

PLUGINLIB_DECLARE_CLASS(astra_depth_filters, DepthFilterNodelet, DepthFilterNodelet, nodelet::Nodelet);