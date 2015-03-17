#include "AbstractPointProcessor.h"
#include <pcl/segmentation/segment_differences.h>

class StaticBackgroundRemover : AbstractPointProcessor {

private:
	CloudConstPtr backGroundCloud;
	pcl::SegmentDifferences<PointType> sd;

public:
	void processData() override;

	void getBackGroundCloud();

	void setBackGroundCloud(CloudConstPtr backGroundCloud);
};
