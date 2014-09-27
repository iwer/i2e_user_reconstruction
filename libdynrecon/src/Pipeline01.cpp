#include "Pipeline01.h"


Pipeline01::Pipeline01(boost::signals2::signal<void (float)> * minDepUpdate, 
					   boost::signals2::signal<void (float)> * maxDepUpdate, 
					   boost::signals2::signal<void (int)> * triangleSizeUpdate)
{


	pp_ = &d;
	mp_ = &m;

	minDepUpdate->connect(boost::bind(&DepthThreshold::setDepthThresholdMin, &d, _1));
	maxDepUpdate->connect(boost::bind(&DepthThreshold::setDepthThresholdMax, &d, _1));
	triangleSizeUpdate->connect(boost::bind(&OrganizedFastMeshProcessor::setEdgeLength, &m, _1));
}


Pipeline01::~Pipeline01(void)
{
}

void Pipeline01::processData()
{
	pp_->setInputCloud(cloud_);
	pp_->processData();
	mp_->setInputCloud(pp_->getOutputCloud());
	mp_->processData();
	mesh_ = mp_->getOutputMesh();
}