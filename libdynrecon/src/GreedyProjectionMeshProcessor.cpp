#include "GreedyProjectionMeshProcessor.h"


GreedyProjectionMeshProcessor::GreedyProjectionMeshProcessor(void)
	: tree (new pcl::search::KdTree<PointType>)
//	, normals (new pcl::PointCloud<pcl::Normal>)
{
}


GreedyProjectionMeshProcessor::~GreedyProjectionMeshProcessor(void)
{
}

void GreedyProjectionMeshProcessor::processData() 
{
}
