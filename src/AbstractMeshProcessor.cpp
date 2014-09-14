#include "AbstractMeshProcessor.h"


AbstractMeshProcessor::AbstractMeshProcessor(void)
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