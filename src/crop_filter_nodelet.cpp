#include "astra_depth_filters/crop_filter_nodelet.h"
#include <pluginlib/class_list_macros.h>

void CropFilterNodelet::onInit()
{
  ROS_INFO("Initializing Crop Filter Nodelet");

  ros::NodeHandle nh = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  filter_.reset(new CropFilter(nh, nh_private));
}

PLUGINLIB_DECLARE_CLASS(astra_depth_filters, CropFilterNodelet, CropFilterNodelet, nodelet::Nodelet);
