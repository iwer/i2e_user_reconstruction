#include "AbstractPointProcessor.h"


AbstractPointProcessor::AbstractPointProcessor(void) :
	inputCloud_(new Cloud),
	outputCloud_(new Cloud)
{
}


AbstractPointProcessor::~AbstractPointProcessor(void)
{
}

void AbstractPointProcessor::setInputCloud(CloudConstPtr cloud)
{
	inputCloud_ = cloud;
}

CloudPtr AbstractPointProcessor::getOutputCloud()
{
	return outputCloud_;
}
