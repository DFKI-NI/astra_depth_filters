#include "astra_depth_filters/artifact_filter.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "temp_noise_filter");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  ArtifactFilter af(nh, nh_priv);
  ros::spin();
  return 0;
}