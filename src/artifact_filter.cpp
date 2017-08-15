#include "artifact_filter.h"

ArtifactFilter::ArtifactFilter(ros::NodeHandle n) :
  n_(n),
  depth_it_(n_),
  numSubscribers(0)
{
  image_transport::SubscriberStatusCallback itsscConnect = boost::bind(&ArtifactFilter::connectCb, this);
  image_transport::SubscriberStatusCallback itsscDisc = boost::bind(&ArtifactFilter::discCb, this);
  image_pub_ = depth_it_.advertise("image_raw_filtered", 1, itsscConnect, itsscDisc);

  dynamic_reconfigure::Server<astra_depth_filters::ArtifactFilterConfig>::CallbackType f;
  f = boost::bind(&ArtifactFilter::reconfigure, this , _1, _2);
  server_.setCallback(f);
}
void ArtifactFilter::connectCb()
{
  if (numSubscribers == 0)
  {
    sub_ = depth_it_.subscribe("image_raw", 1, &ArtifactFilter::processDepthImage, this);
  }
  //ROS_INFO("ArtifactFilter Running");
  ++numSubscribers;
}

void ArtifactFilter::discCb()
{
  if (image_pub_.getNumSubscribers() == 0)
  {
    numSubscribers = 0;
    sub_.shutdown();
    //ROS_INFO("ArtifactFilter shutting down...");
  }
}

void ArtifactFilter::reconfigure(astra_depth_filters::ArtifactFilterConfig &dfconfig, uint32_t level)
{
  config_ = dfconfig;
}

/*
 * Processes the depth image
 *
 *
 */
void ArtifactFilter::processDepthImage(const sensor_msgs::ImageConstPtr& dimg)
{
  if (!config_.enable)
  {
    image_pub_.publish(dimg);
    return;
  }

  int outputEncoding;

  cv_bridge::CvImagePtr image_in;
  //output image
  cv_bridge::CvImage image_out;
  image_out.header = dimg->header;
  image_out.encoding = dimg->encoding;

  try
  {
    //converts image to 32bitfloat format
    image_in = cv_bridge::toCvCopy(dimg, "32FC1"/*TYPE*/);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("CV Bridge Error: %s", e.what());
    return;
  }


  if (std::strncmp(dimg->encoding.c_str(), "32FC1", 6) == 0) //returns 0 if equal.........
  {
    outputEncoding = CV_32FC1;
    image_in->image *= 1000.0; //convert to mm scale
  }
  else if (std::strncmp(dimg->encoding.c_str(), "16UC1", 6) == 0)
  {
    outputEncoding = CV_16UC1;
  }

  filterArtifact(image_in->image);


  if (outputEncoding == CV_32FC1)
  {
    image_in->image /= 1000;
    //set zeros to NaN
    image_in->image.setTo(std::numeric_limits<double>::quiet_NaN(), image_in->image == 0);
  }

  image_in->image.convertTo(image_out.image, outputEncoding);

  image_pub_.publish(image_out.toImageMsg());
}

void ArtifactFilter::filterArtifact(cv::Mat image)
{
  cv::Mat mask(image.rows, image.cols, CV_8U);
  cv::inRange(image, config_.min_z, config_.max_z, mask);
  mask = (mask - 1) * (-1);
  cv::multiply(image, mask, image, 1, CV_32FC1);
}