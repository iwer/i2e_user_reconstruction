#pragma once
#include "AbstractPointProcessor.h"
#include <pcl/filters/passthrough.h>
#include "typedefs.h"

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

