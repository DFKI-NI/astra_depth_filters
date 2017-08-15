#include "artifact_filter.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "temp_noise_filter");
  ros::NodeHandle n;
  ArtifactFilter af(n);
  ros::spin();
  return 0;
}