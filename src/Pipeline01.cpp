#include "Pipeline01.h"


Pipeline01::Pipeline01(void)
{
	pp_ = new DepthThreshold();
	mp_ = new OrganizedFastMeshProcessor();
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