#include "AbstractProcessingPipeline.h"


AbstractProcessingPipeline::AbstractProcessingPipeline(void)
{
}


AbstractProcessingPipeline::~AbstractProcessingPipeline(void)
{
}

void AbstractProcessingPipeline::setInputCloud(CloudConstPtr cloud){
	cloud_ = cloud;
}

ofMesh * AbstractProcessingPipeline::getOutputMesh(){
	return &mesh_;
}