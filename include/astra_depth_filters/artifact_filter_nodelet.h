#ifndef ASTRA_DEPTH_FILTERS_ARTIFACT_FILTER_H
#define ASTRA_DEPTH_FILTERS_ARTIFACT_FILTER_H

#include <nodelet/nodelet.h>

#include "astra_depth_filters/artifact_filter.h"

class ArtifactFilterNodelet : public nodelet::Nodelet
{
public:
  virtual void onInit();

private:
  boost::shared_ptr<ArtifactFilter> filter_;
};

#endif // ASTRA_DEPTH_FILTERS_ARTIFACT_FILTER_H