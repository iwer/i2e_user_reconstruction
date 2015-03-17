#pragma once
#include "AbstractPointProcessor.h"
#include <pcl/filters/passthrough.h>
#include "typedefs.h"

/**
  * Point Cloud Thresholdfilter, configured on Z axis with adjustable min and max ranges. Preserves "organized" constraint in Point Clouds 
  */
class DepthThreshold :
	public AbstractPointProcessor
{
public:
	DepthThreshold(void);
	~DepthThreshold(void);

	void processData() override;

	float getDepthThresholdMax();
	void setDepthThresholdMax(float value);
	float getDepthThresholdMin();
	void setDepthThresholdMin(float value);
private:
	pcl::PassThrough<PointType> pass_;
	float depthThreshMax;
	float depthThreshMin;

};

