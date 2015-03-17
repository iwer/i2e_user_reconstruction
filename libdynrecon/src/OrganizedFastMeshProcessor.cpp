#include "OrganizedFastMeshProcessor.h"


OrganizedFastMeshProcessor::OrganizedFastMeshProcessor(void) :
	AbstractMeshProcessor(){
	ofm.setTriangulationType (pcl::OrganizedFastMesh<PointType>::TRIANGLE_LEFT_CUT);
	ofmPixelSize = 5;
}


OrganizedFastMeshProcessor::~OrganizedFastMeshProcessor(void)
{
}

void OrganizedFastMeshProcessor::processData()
{
	if(inputCloud_->size() > 0) {
		// fast organized mesh triangulation
		std::cout << "Cloud Size before Meshing: " << inputCloud_->points.size() << std::endl;

		ofm.setTrianglePixelSize (ofmPixelSize);
		ofm.setInputCloud(inputCloud_);
		{
			boost::mutex::scoped_lock(triangle_mutex_);
			ofm.reconstruct (*triangles_);
		}
		ofm.getIndices();
		//std::cout << "Reconstructed triangles: " << triangles_->size() << std::endl;

	}
}

int OrganizedFastMeshProcessor::getEdgeLength()
{
	return ofmPixelSize;
}

void OrganizedFastMeshProcessor::setEdgeLength(int value)
{
	ofmPixelSize = value;
}

