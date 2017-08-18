#include "astra_depth_filters/depth_filter.h"

DepthFilter::DepthFilter(ros::NodeHandle nh, ros::NodeHandle nh_priv) :
  nh_(nh),
  nh_priv_(nh_priv),
  depth_it_(nh_),
  numSubscribers(0),
  time_running(0),
  server_(nh_priv_)
{

  image_transport::SubscriberStatusCallback itsscConnect = boost::bind(&DepthFilter::connectCb, this);
  image_transport::SubscriberStatusCallback itsscDisc = boost::bind(&DepthFilter::discCb, this);
  image_pub_ = depth_it_.advertise("image_raw_filtered", 10, itsscConnect, itsscDisc);
  // image_debug_pub_ = depth_it_.advertise("depth_filter_debug_image", 1);

  dynamic_reconfigure::Server<astra_depth_filters::DepthFilterConfig>::CallbackType f;
  f = boost::bind(&DepthFilter::reconfigure, this , _1, _2);
  server_.setCallback(f);
}

void DepthFilter::connectCb()
{
  if (numSubscribers == 0)
  {
    sub_ = depth_it_.subscribe("image_raw", 1, &DepthFilter::processDepthImage, this);
  }
  //ROS_INFO("DepthFilter Running");
  ++numSubscribers;
}

void DepthFilter::discCb()
{
  if (image_pub_.getNumSubscribers() == 0)
  {
    numSubscribers = 0;
    sub_.shutdown();
    //ROS_INFO("DepthFilter shutting down...");
  }
}

void DepthFilter::reconfigure(astra_depth_filters::DepthFilterConfig &dfconfig, uint32_t level)
{
  if (dfconfig.laplace_kernel_size % 2 == 0)
  {
    // laplaceKernelSize is even -> reset to previous value
    dfconfig.laplace_kernel_size = config_.laplace_kernel_size;
  }
  config_ = dfconfig;
}

/*
 * Processes the depth image
 *
 *
 */
void DepthFilter::processDepthImage(const sensor_msgs::ImageConstPtr& dimg)
{
  if (!config_.enable)
  {
    image_pub_.publish(dimg);
    return;
  }

  // clock_t start = clock();

  int outputEncoding;

  cv_bridge::CvImagePtr image_in;
  //output image
  cv_bridge::CvImage image_out;
  image_out.header = dimg->header;
  image_out.encoding = dimg->encoding;

  //converts image to 32bitfloat format
  try
  {
    image_in = cv_bridge::toCvCopy(dimg, "32FC1"/*TYPE*/);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("CV Bridge Error: %s", e.what());
    return;
  }
  //there are NaNs in the float image! (depth_registered)
  if (std::strncmp(dimg->encoding.c_str(), "32FC1", 6) == 0) //returns 0 if equal.........
  {
    outputEncoding = CV_32FC1;
    removeNaNs(image_in->image);
    image_in->image *= 1000.0; //convert to mm scale
  }
  else if (std::strncmp(dimg->encoding.c_str(), "16UC1", 6) == 0)
  {
    outputEncoding = CV_16UC1;
  }

  filter(image_in->image);

  //revert to meter scale
  if (outputEncoding == CV_32FC1)
  {
    image_in->image /= 1000;
  }

  insertNaNs(image_in->image);

  image_in->image.convertTo(image_out.image, outputEncoding);
  image_pub_.publish(image_out.toImageMsg());

  //time for one frame
  // clock_t end = clock();

  // double elapsed_secs = double(end - start) / CLOCKS_PER_SEC;
  //time_running += elapsed_secs;
  // ROS_INFO("Time for frame: %f", elapsed_secs);
  //ROS_INFO("Time for frame: %f", time_running);
}

/*
* replaces all NaNs with zeros
*/
void DepthFilter::removeNaNs(cv::Mat image)
{
  for (int i = 0 ; i < image.rows ; i++)
  {
    for (int j = 0 ; j < image.cols ; j++)
    {
      if (std::isnan(image.at<float>(i, j)))
      {
        image.at<float>(i, j) = 0;
      }
    }
  }
}
/*
* replaces all zeros with NaNs
*/
void DepthFilter::insertNaNs(cv::Mat image)
{
  image.setTo(std::numeric_limits<double>::quiet_NaN(), image == 0);
}
/*
* filters the depth image by applying a laplace filter, dilating the resulting edge mask and then masking the remaining pixels
*
*/
void DepthFilter::filter(cv::Mat image)
{
  //find edges in the depth image
  cv::Mat edges;
  cv::Laplacian(image, edges, CV_32FC1, config_.laplace_kernel_size);
  //convert to binary (high values at edges-> set everything above threshold to 1, rest 0)
  cv::threshold(edges, edges, config_.filter_threshold, 1, CV_THRESH_BINARY);
  //broaden the edges
  cv::Mat structElem = cv::getStructuringElement(config_.struct_shape, cv::Size(config_.dilate_struct_size, config_.dilate_struct_size));
  cv::dilate(edges, edges, structElem);

  //invert binary image
  edges = (edges - 1) * (-1);

  //call the similarFilter
  similarFilter(image, edges);

  //edges = (edges - 1) * (-1); shows noise instead of image

  //int invalidPixels = cv::countNonZero(edges);
  //ROS_INFO("Number of Pixels deleted: %i from %i", (edges.rows * edges.cols) - invalidPixels, edges.rows * edges.cols);

  //use edges to mask out the garbagepixels
  cv::multiply(edges, image, image);


  // cv_bridge::CvImage image_debug;
  // image_debug.header = image_in->header;
  // image_debug.encoding = "32FC1";
  // image_debug.image = edges;
  // image_debug_pub_.publish(image_debug.toImageMsg());

}

/*
* Filters pixels on edges: if pixels are similar to the neighbours, they will be taken into the result
*
*/
void DepthFilter::similarFilter(cv::Mat img, cv::Mat edges)
{
  if (!config_.similar_filter)
  {
    return;
  }
  for (int i = 1 ; i < (img.rows) - 1 ; i++)
  {
    for (int j = 1 ; j < (img.cols) - 1 ; j++)
    {
      if (edges.at<float>(i, j) == 0) //for all edge points we detected: should we use them or throw them away..?
      {
        int similarPixels = 0;
        //get a submatrix around middle pixel -> neighbourhood
        cv::Mat submat(img, cv::Rect(j - 1, i - 1, 3, 3));

        //iterate through neighbourhood
        cv::MatConstIterator_<float> it = submat.begin<float>(), it_end = submat.end<float>();
        for (; it != it_end; ++it)
        {
          //if pixel distance to neighbour < distThresh -> they count as similar
          if (fabs(submat.at<float>(1, 1) - *it) < config_.dist_thresh)
          {
            similarPixels++;
          }
        }
        similarPixels--; //middle pixel is similar to itself
        //if there are enough similar pixels nearby, we enable that pixel through the edge matrix
        if (similarPixels > config_.similar_thresh)
        {
          edges.at<float>(i, j) = 1;
        }
      }
    }
  }
}