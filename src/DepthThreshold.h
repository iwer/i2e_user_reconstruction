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

	void processData();
	void guiEvent(ofxUIEventArgs &e);
private:
	pcl::PassThrough<PointType> pass_;
	float depthThreshMax;
	float depthThreshMin;

};

