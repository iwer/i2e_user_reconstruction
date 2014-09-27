#pragma once
#include "AbstractProcessingPipeline.h"
#include "DepthThreshold.h"
#include "OrganizedFastMeshProcessor.h"
#include <boost/signals2.hpp>

class Pipeline01 :
	public AbstractProcessingPipeline
{
public:
	Pipeline01(boost::signals2::signal<void (float)> * minDepUpdate, 
		boost::signals2::signal<void (float)> * maxDepUpdate, 
		boost::signals2::signal<void (int)> * triangleSizeUpdate);
	~Pipeline01(void);

	void processData();

	DepthThreshold d;
	OrganizedFastMeshProcessor m;
};

