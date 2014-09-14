#pragma once
#include "AbstractProcessingPipeline.h"
#include "DepthThreshold.h"
#include "OrganizedFastMeshProcessor.h"
class Pipeline01 :
	public AbstractProcessingPipeline
{
public:
	Pipeline01(void);
	~Pipeline01(void);

	void processData();

};

