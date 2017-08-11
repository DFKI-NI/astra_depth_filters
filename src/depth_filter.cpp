#include "depth_filter.h"

DepthFilter::DepthFilter() :
  depth_it_(n_)
{
  sub_ = depth_it_.subscribe("image_raw", 1, &DepthFilter::processDepthImage, this);
  image_pub_ = depth_it_.advertise("image_raw_filtered", 1);
  image_debug_pub_ = depth_it_.advertise("depth_filter_debug_image", 1);

  dynamic_reconfigure::Server<astra_depth_filters::DepthFilterConfig>::CallbackType f;
  f = boost::bind(&DepthFilter::reconfigure, this , _1, _2);
  server_.setCallback(f);
}

void DepthFilter::reconfigure(astra_depth_filters::DepthFilterConfig &dfconfig, uint32_t level)
{
  if (dfconfig.laplaceKernelSize % 2 == 0)
  {
    // laplaceKernelSize is even -> reset to previous value
    dfconfig.laplaceKernelSize = config_.laplaceKernelSize;
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

  clock_t start = clock();

  cv_bridge::CvImagePtr image_in;
  std_msgs::Header orig_header = dimg->header;
  //output image
  cv_bridge::CvImage image_out;
  image_out.header = orig_header;
  image_out.encoding = "16UC1";
  //debug image
  cv_bridge::CvImage image_debug;
  image_debug.header = orig_header;
  image_debug.encoding = "32FC1";
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
  //std::cout << laplaceKernelSize << " " << filterThreshold << " " << dilateStructSize <<std::endl;

  //find edges in the depth image
  cv::Mat edges;
  cv::Laplacian(image_in->image, edges, CV_32F, config_.laplaceKernelSize);

  //convert to binary (high values at edges-> set everything above threshold to 1, rest 0)
  cv::threshold(edges, edges, config_.filterThreshold, 1, CV_THRESH_BINARY);
  //broaden the edges
  cv::Mat structElem = cv::getStructuringElement(config_.structShape, cv::Size(config_.dilateStructSize, config_.dilateStructSize));
  cv::dilate(edges, edges, structElem);

  //remove the border
  //cv::Rect border(cv::Point(0, 0), edges.size());
  //cv::Scalar color(0, 0, 0);
  //int thickness = 100;
  //cv::rectangle(edges, border, color, thickness);

  //invert binary image


  edges = (edges - 1) * (-1);

  //try something!

  similarFilter(image_in->image, edges);

  //edges = (edges - 1) * (-1); shows noise instead of image

  int invalidPixels = cv::countNonZero(edges);
  ROS_INFO("Number of Pixels deleted: %i from %i", (edges.rows * edges.cols) - invalidPixels, edges.rows * edges.cols);
  //use edges to mask out the garbagepixels
  cv::multiply(edges, image_in->image, image_in->image);



  image_in->image.convertTo(image_out.image, CV_16UC1);

  double min, max;
  cv::minMaxLoc(image_out.image, &min, &max);
  ROS_INFO("Min: %f ; Max: %f", min, max);
  //time for one frame
  clock_t end = clock();
  double elapsed_secs = double(end - start) / CLOCKS_PER_SEC;
  ROS_INFO("Time for frame: %f", elapsed_secs);


  image_debug.image = edges;
  image_debug_pub_.publish(image_debug.toImageMsg());
  image_pub_.publish(image_out.toImageMsg());

}
/*
* Filters pixels on edges: if pixels are similar to the neighbours, they will be taken into the result
*
*/
void DepthFilter::similarFilter(cv::Mat img, cv::Mat edges)
{
  if (!config_.similarFilter)
  {
    return;
  }
  for (int i = 1 ; i < (img.rows) - 1 ; i++)
  {
    for (int j = 1 ; j < (img.cols) - 1 ; j++)
    {
      if (edges.at<float>(i, j) == 0)
      {
        int similarPixels = 0;
        cv::Mat submat(img, cv::Rect(j - 1, i - 1, 3, 3));
        submat = submat.clone(); //for continuosity (isContinuous)
        submat.reshape(0, 1); //so we get a vector to iterate on
        for (int neighbour = 0 ; neighbour < 9 ; neighbour++)
        {

          if (neighbour != 4) //dont compare the middle pixel to itself
          {

            if (fabs(submat.at<float>(4) - submat.at<float>(neighbour)) < config_.distThresh)
            {
              similarPixels++;
            }
          }
        }
        if (similarPixels > config_.similarThresh)
        {
          edges.at<float>(i, j) = 1;
        }
      }
    }
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "depth_filter");
  DepthFilter df;
  ros::spin();
  return 0;
}
