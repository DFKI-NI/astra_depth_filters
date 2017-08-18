#include "astra_depth_filters/noise_filter.h"
//noise filter node

int main(int argc, char **argv)
{
  ros::init(argc, argv, "noise_filter");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  NoiseFilter nf(nh, nh_priv);
  ros::spin();
  return 0;
}