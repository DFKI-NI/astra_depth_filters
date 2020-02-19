#ifndef ASTRA_DEPTH_FILTERS_CROP_FILTER_H
#define ASTRA_DEPTH_FILTERS_CROP_FILTER_H

#include <nodelet/nodelet.h>

#include "astra_depth_filters/crop_filter.h"

class CropFilterNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  boost::shared_ptr<CropFilter> filter_;
};

#endif // ASTRA_DEPTH_FILTERS_CROP_FILTER_H
