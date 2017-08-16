#ifndef ASTRA_DEPTH_FILTERS_DEPTH_FILTER_H
#define ASTRA_DEPTH_FILTERS_DEPTH_FILTER_H

#include <nodelet/nodelet.h>

#include "astra_depth_filters/depth_filter.h"

class DepthFilterNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  boost::shared_ptr<DepthFilter> filter_;
};

#endif // ASTRA_DEPTH_FILTERS_DEPTH_FILTER_H