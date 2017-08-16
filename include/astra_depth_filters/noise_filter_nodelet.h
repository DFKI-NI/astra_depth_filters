#ifndef ASTRA_DEPTH_FILTERS_NOISE_FILTER_H
#define ASTRA_DEPTH_FILTERS_NOISE_FILTER_H

#include <nodelet/nodelet.h>

#include "astra_depth_filters/noise_filter.h"

class NoiseFilterNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  boost::shared_ptr<NoiseFilter> filter_;
};

#endif // ASTRA_DEPTH_FILTERS_NOISE_FILTER_H