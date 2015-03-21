#pragma once
#include "AbstractProcessingPipeline.h"
#include "DepthThreshold.h"
#include "GreedyProjectionMeshProcessor.h"
#include <boost/signals2.hpp>
#include "PointCloudSampler.h"

class Pipeline02 :
	public AbstractProcessingPipeline
{
public:
	Pipeline02(boost::signals2::signal<void (float)> * minDepUpdate, 
		boost::signals2::signal<void (float)> * maxDepUpdate, 
		boost::signals2::signal<void (float)> * triangleSizeUpdate);
	~Pipeline02(void);

	void processData();

private:
	DepthThreshold d;
	PointCloudSampler s;
	GreedyProjectionMeshProcessor g;
};

