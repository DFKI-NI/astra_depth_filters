#include "astra_depth_filters/artifact_filter_nodelet.h"
#include <pluginlib/class_list_macros.h>

void ArtifactFilterNodelet::onInit()
{
  ROS_INFO("Initializing Artifact Filter Nodelet");

  // TODO: Do we want the single threaded or multithreaded NH?
  ros::NodeHandle nh = getMTNodeHandle();
  ros::NodeHandle nh_private = getMTPrivateNodeHandle();

  filter_.reset(new ArtifactFilter(nh, nh_private));
}

PLUGINLIB_EXPORT_CLASS(ArtifactFilterNodelet, nodelet::Nodelet);
