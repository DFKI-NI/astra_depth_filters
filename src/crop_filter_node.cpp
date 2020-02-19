#include "astra_depth_filters/crop_filter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crop_filter");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  CropFilter af(nh, nh_priv);
  ros::spin();
  return 0;
}
