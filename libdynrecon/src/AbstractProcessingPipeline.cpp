#include "AbstractProcessingPipeline.h"


AbstractProcessingPipeline::AbstractProcessingPipeline(void) 
	: cloud_(new Cloud)
	, meshCloud_(new Cloud)
	, triangles_(new std::vector<pcl::Vertices>)
{
}


AbstractProcessingPipeline::~AbstractProcessingPipeline(void)
{
}

void AbstractProcessingPipeline::setInputCloud(CloudConstPtr cloud){
	cloud_ = nullptr;
	cloud_ = cloud;
}

CloudConstPtr AbstractProcessingPipeline::getInputCloud()
{
	return meshCloud_;
}

TrianglesPtr AbstractProcessingPipeline::getTriangles()
{
	return triangles_;
}
