#include "depth_filter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth_filter");
  ros::NodeHandle n;
  DepthFilter df(n);
  ros::spin();
  return 0;
}
