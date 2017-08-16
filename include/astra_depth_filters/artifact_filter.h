#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <astra_depth_filters/ArtifactFilterConfig.h>
#include <ctime>

class ArtifactFilter
{
  ros::NodeHandle nh_;

  ros::NodeHandle nh_priv_;

  image_transport::ImageTransport depth_it_;

  image_transport::Publisher image_pub_;

  image_transport::Subscriber sub_;

  astra_depth_filters::ArtifactFilterConfig config_;

  dynamic_reconfigure::Server<astra_depth_filters::ArtifactFilterConfig> server_;

  int numSubscribers;

  void connectCb();

  void discCb();

  void filterArtifact(cv::Mat image);

  void reconfigure(astra_depth_filters::ArtifactFilterConfig &dfconfig, uint32_t level);

  void processDepthImage(const sensor_msgs::ImageConstPtr& dimg);
public:
  ArtifactFilter(ros::NodeHandle nh, ros::NodeHandle nh_priv);


};