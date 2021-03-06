#include "astra_depth_filters/noise_filter.h"

NoiseFilter::NoiseFilter(ros::NodeHandle nh, ros::NodeHandle nh_priv) :
  nh_(nh),
  nh_priv_(nh_priv),
  depth_it_(nh_),
  numSubscribers(0),
  server_(nh_priv_)
{
  image_transport::SubscriberStatusCallback itsscConnect = boost::bind(&NoiseFilter::connectCb, this);
  image_transport::SubscriberStatusCallback itsscDisc = boost::bind(&NoiseFilter::discCb, this);
  image_pub_ = depth_it_.advertise("image_raw_filtered", 10, itsscConnect, itsscDisc);

  dynamic_reconfigure::Server<astra_depth_filters::NoiseFilterConfig>::CallbackType f;
  f = boost::bind(&NoiseFilter::reconfigure, this , _1, _2);
  server_.setCallback(f);
}

void NoiseFilter::connectCb()
{
  if (numSubscribers == 0)
  {
    sub_ = depth_it_.subscribe("image_raw", 1, &NoiseFilter::processDepthImage, this);
  }
  // ROS_INFO("NoiseFilter Running");
  ++numSubscribers;
}

void NoiseFilter::discCb()
{
  if (image_pub_.getNumSubscribers() == 0)
  {
    numSubscribers = 0;
    sub_.shutdown();
    // ROS_INFO("NoiseFilter shutting down...");
  }
}

void NoiseFilter::reconfigure(astra_depth_filters::NoiseFilterConfig &dfconfig, uint32_t level)
{
  config_ = dfconfig;
}

/*
 * Processes the depth image
 *
 *
 */
void NoiseFilter::processDepthImage(const sensor_msgs::ImageConstPtr& dimg)
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

  filter(image_in->image);


  if (outputEncoding == CV_32FC1)
  {
    image_in->image /= 1000;
    //set zeros to NaN
    image_in->image.setTo(std::numeric_limits<double>::quiet_NaN(), image_in->image == 0);
  }

  image_in->image.convertTo(image_out.image, outputEncoding);

  image_pub_.publish(image_out.toImageMsg());
}

/*
* applies a noisefilter that iterates over rows, adds up the absolute differences and setting rows to zero which surpass threshold
*
*/
void NoiseFilter::filter(cv::Mat image)
{
  int noiseRows = 0;
  int row = 0;
  for (; row < image.rows; row++)
  {
    const float* Mi = image.ptr<float>(row); //get pointer on row for easy []-access
    float rowdiff = 0;
    float artifactDiff = 0;
    for (int j = 0; j < image.cols - 1; j++) //sum up all the differences per row
    {
      if (std::isnormal(Mi[j]) && std::isnormal(Mi[j + 1]))
      {
        float diff = Mi[j] - Mi[j + 1];
        rowdiff += fabs(diff);
        //this tries to recognize the static noisy artifacts. parts of rows have strictly decreasing values with big differences.
        //if consecutive differences are too high, the row will be removed
        if (diff > config_.artifact_diff)
        {
          artifactDiff += 1;
        }

        if (diff < 0)
        {
          artifactDiff = 0;
        }
        if (artifactDiff > config_.artifact_length)
        {
          image.row(row).setTo(0);
          break;
        }
      }
    }
    //and compare them to the threshold
    if (rowdiff > config_.diff_thresh)
    {
      noiseRows++;
    }
    else
    {
      noiseRows = 0; //only consectutive rows count
    }

    if (noiseRows > config_.min_noise_rows)
    {
      row -= (noiseRows - 1) + config_.add_rows; //remove all rows starting from the first in the set
      break;
    }
  }
  //set all lower rows to 0 starting at row
  for (; noiseRows > config_.min_noise_rows &&  row < image.rows; row++)
  {
    image.row(row).setTo(0);
  }
}