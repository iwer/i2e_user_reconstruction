#include "PointCloudSampler.h"

PointCloudSampler::PointCloudSampler() 
	: AbstractPointProcessor()
{
	resolution_ = .1; // 10 cm
}

PointCloudSampler::~PointCloudSampler()
{
}

void PointCloudSampler::processData() {
	if(inputCloud_->size() > 0)
	{
		vg.setLeafSize(resolution_, resolution_, resolution_);
		vg.setInputCloud(inputCloud_);
		vg.filter(*outputCloud_);
	}
}

float PointCloudSampler::getResolution() const
{
	return resolution_;
}

void PointCloudSampler::setResolution(float resolution)
{
	resolution_ = resolution;
}