#include "astra_depth_filters/depth_filter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_filter");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  DepthFilter df(nh, nh_priv);
  ros::spin();
  return 0;
}
