#include "GreedyProjectionMeshProcessor.h"


GreedyProjectionMeshProcessor::GreedyProjectionMeshProcessor(void)
	: tree (new pcl::search::KdTree<PointType>)
	, normals (new pcl::PointCloud<pcl::Normal>)

{
}


GreedyProjectionMeshProcessor::~GreedyProjectionMeshProcessor(void)
{
}

void GreedyProjectionMeshProcessor::processData() 
{
	// Normal estimation*
	tree->setInputCloud (inputCloud_);
	n.setInputCloud (inputCloud_);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);

	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<PointNormalType>::Ptr cloud_with_normals (new pcl::PointCloud<PointNormalType>);
	pcl::concatenateFields (*inputCloud_.get(), *normals.get(), *cloud_with_normals.get());
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<PointNormalType>::Ptr tree2 (new pcl::search::KdTree<PointNormalType>);
	tree2->setInputCloud (cloud_with_normals);

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.025);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (*triangles_);

	// Additional vertex information
	auto parts = gp3.getPartIDs();
	auto states = gp3.getPointStates();
}
