#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <astra_depth_filters/DepthFilterConfig.h>
#include <ctime>
class DepthFilter
{
  ros::NodeHandle n_;

  image_transport::ImageTransport depth_it_;

  image_transport::Publisher image_pub_;

  image_transport::Publisher image_debug_pub_;

  image_transport::Subscriber sub_;

  astra_depth_filters::DepthFilterConfig config_;

  dynamic_reconfigure::Server<astra_depth_filters::DepthFilterConfig> server_;

  double time_running;

  int numSubscribers;

  void connectCb();

  void discCb();

  void removeNaNs(cv::Mat image);

  void insertNaNs(cv::Mat image);

  void filter(cv::Mat image);

  void reconfigure(astra_depth_filters::DepthFilterConfig &dfconfig, uint32_t level);

  void processDepthImage(const sensor_msgs::ImageConstPtr& dimg);

  void similarFilter(cv::Mat img, cv::Mat edges);
public:

  DepthFilter(ros::NodeHandle n);


};