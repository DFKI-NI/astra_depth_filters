#include "astra_depth_filters/crop_filter.h"

CropFilter::CropFilter(ros::NodeHandle nh, ros::NodeHandle nh_priv) :
    nh_(nh),
    nh_priv_(nh_priv),
    depth_it_(nh_),
    numSubscribers(0),
    server_(nh_priv_)
{
  image_transport::SubscriberStatusCallback itsscConnect = boost::bind(&CropFilter::connectCb, this);
  image_transport::SubscriberStatusCallback itsscDisc = boost::bind(&CropFilter::discCb, this);
  image_pub_ = depth_it_.advertise("image_raw_filtered", 10, itsscConnect, itsscDisc);

  dynamic_reconfigure::Server<astra_depth_filters::CropFilterConfig>::CallbackType f;
  f = boost::bind(&CropFilter::reconfigure, this, _1, _2);
  server_.setCallback(f);
}
void CropFilter::connectCb()
{
  if (numSubscribers == 0)
  {
    sub_ = depth_it_.subscribe("image_raw", 1, &CropFilter::processDepthImage, this);
  }
  ++numSubscribers;
}

void CropFilter::discCb()
{
  if (image_pub_.getNumSubscribers() == 0)
  {
    numSubscribers = 0;
    sub_.shutdown();
  }
}

void CropFilter::reconfigure(astra_depth_filters::CropFilterConfig &config, uint32_t)
{
  if (config.max_x < config.min_x)
    config.max_x = config.min_x;
  if (config.max_y < config.min_y)
    config.max_y = config.min_y;
  config_ = config;
}

void CropFilter::processDepthImage(const sensor_msgs::ImageConstPtr &dimg)
{
  if (!config_.enable)
  {
    image_pub_.publish(dimg);
    return;
  }

  int outputEncoding;

  cv_bridge::CvImagePtr image_in;
  cv_bridge::CvImage image_out;
  image_out.header = dimg->header;
  image_out.encoding = dimg->encoding;

  try
  {
    //converts image to 32bitfloat format
    image_in = cv_bridge::toCvCopy(dimg, "32FC1"/*TYPE*/);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("CV Bridge Error: %s", e.what());
    return;
  }

  if (std::strncmp(dimg->encoding.c_str(), "32FC1", 6) == 0) //returns 0 if equal.........
  {
    outputEncoding = CV_32FC1;
    image_in->image *= 1000.0; //convert to mm scale
  } else if (std::strncmp(dimg->encoding.c_str(), "16UC1", 6) == 0)
  {
    outputEncoding = CV_16UC1;
  }

  filterCrop(image_in->image);

  if (outputEncoding == CV_32FC1)
  {
    image_in->image /= 1000;
    //set zeros to NaN
    image_in->image.setTo(std::numeric_limits<double>::quiet_NaN(), image_in->image == 0);
  }

  image_in->image.convertTo(image_out.image, outputEncoding);

  image_pub_.publish(image_out.toImageMsg());
}

void CropFilter::filterCrop(cv::Mat image)
{
  int x = config_.min_x;
  int y = config_.min_y;
  int width = config_.max_x - config_.min_x;
  int height = config_.max_y - config_.min_y;

  cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8U);
  mask(cv::Rect(x, y, width, height)) = 1;
  cv::multiply(image, mask, image, 1, CV_32FC1);
}
