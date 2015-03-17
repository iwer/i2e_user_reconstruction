#pragma once
#include "typedefs.h"
#include "AbstractMeshProcessor.h"
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>

class GreedyProjectionMeshProcessor :
	public AbstractMeshProcessor
{
public:
	GreedyProjectionMeshProcessor(void);
	~GreedyProjectionMeshProcessor(void);

	void processData() override;

private:
	pcl::NormalEstimationOMP<PointType, pcl::Normal> n;
	pcl::search::KdTree<PointType>::Ptr tree;
	pcl::GreedyProjectionTriangulation<PointType> gp3;

	pcl::PointCloud<pcl::Normal>::Ptr normals;
};

