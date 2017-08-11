#include "noise_filter.h"

NoiseFilter::NoiseFilter() :
  depth_it_(n_)
{
  sub_ = depth_it_.subscribe("camera/depth/image_raw", 1, &NoiseFilter::processDepthImage, this);
  image_pub_ = depth_it_.advertise("camera/depth/image_raw_noisefilter", 1);

  dynamic_reconfigure::Server<astra_depth_filters::NoiseFilterConfig>::CallbackType f;
  f = boost::bind(&NoiseFilter::reconfigure, this , _1, _2);
  server_.setCallback(f);
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

  cv_bridge::CvImagePtr image_in;
  std_msgs::Header orig_header = dimg->header;
  //output image
  cv_bridge::CvImage image_out;
  image_out.header = orig_header;
  image_out.encoding = "16UC1";
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
  

  for (int i = 0; i < image_in->image.rows; i++)
  {
    
    //const float* Mi = image_in->image.ptr<float>(i);
    for ( int step = 0 ; step < image_in->image.cols-config_.filterWidth ; step+= config_.filterWidth)
    {
      float rowdiff = 0;
      cv::Mat submat(image_in->image,cv::Rect(step,i,config_.filterWidth,1));
      //submat=submat.clone();
      const float* Mi =submat.ptr<float>(1);
      for (int j = 0; j < submat.cols-1; j++)
      {
        rowdiff += fabs(Mi[j] - Mi[j+1]); 
      }
      
      if (rowdiff > config_.diffThresh)
      {
        submat.row(0).setTo(0);
  //      ROS_ERROR("submat size: %d", submat.row(0).cols);
      }
    }
  }



  image_in->image.convertTo(image_out.image, CV_16UC1);

  image_pub_.publish(image_out.toImageMsg());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "temp_noise_filter");
  NoiseFilter df;
  ros::spin();
  return 0;
}
