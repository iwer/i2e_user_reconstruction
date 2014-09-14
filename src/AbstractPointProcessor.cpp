#include "AbstractPointProcessor.h"


AbstractPointProcessor::AbstractPointProcessor(void)
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
