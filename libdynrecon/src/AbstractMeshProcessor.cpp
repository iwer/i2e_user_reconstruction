#include "AbstractMeshProcessor.h"


AbstractMeshProcessor::AbstractMeshProcessor(void) :
	inputCloud_(new Cloud)
{
}


AbstractMeshProcessor::~AbstractMeshProcessor(void)
{
}

void AbstractMeshProcessor::setInputCloud(CloudConstPtr cloud)
{
	inputCloud_ = cloud;
}

ofMesh * AbstractMeshProcessor::getOutputMesh()
{
	return &outputMesh_;
}