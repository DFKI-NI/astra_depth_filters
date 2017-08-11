#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <astra_depth_filters/NoiseFilterConfig.h>
#include <ctime>
class NoiseFilter
{
  ros::NodeHandle n_;

  image_transport::ImageTransport depth_it_;

  image_transport::Publisher image_pub_;

  image_transport::Subscriber sub_;

  astra_depth_filters::NoiseFilterConfig config_;

  dynamic_reconfigure::Server<astra_depth_filters::NoiseFilterConfig> server_;

  void reconfigure(astra_depth_filters::NoiseFilterConfig &dfconfig, uint32_t level);

  void processDepthImage(const sensor_msgs::ImageConstPtr& dimg);
public:
  NoiseFilter();


};