#include "noise_filter.h"
//noise filter node

int main(int argc, char **argv)
{
  ros::init(argc, argv, "temp_noise_filter");
  ros::NodeHandle nh;
  NoiseFilter nf(nh);
  ros::spin();
  return 0;
}