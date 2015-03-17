#include "AbstractMeshProcessor.h"


AbstractMeshProcessor::AbstractMeshProcessor(void) :
	inputCloud_(new Cloud),
	triangles_(new std::vector<pcl::Vertices>)
{
}


AbstractMeshProcessor::~AbstractMeshProcessor(void)
{
}

CloudConstPtr AbstractMeshProcessor::getInputCloud()
{

	return inputCloud_;
}

TrianglesPtr AbstractMeshProcessor::getTriangles()
{
	boost::mutex::scoped_lock(triangle_mutex_);
	return triangles_;
}

void AbstractMeshProcessor::setInputCloud(CloudConstPtr cloud)
{
	boost::mutex::scoped_lock(cloud_mutex_);
	inputCloud_ = cloud;
}

