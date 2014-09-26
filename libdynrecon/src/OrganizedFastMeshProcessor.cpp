#include "OrganizedFastMeshProcessor.h"


OrganizedFastMeshProcessor::OrganizedFastMeshProcessor(void) :
	AbstractMeshProcessor(),
	triangles_(new std::vector<pcl::Vertices>)
{
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
		std::cout << "Cloud Size before Meshing: " << inputCloud_->size() << std::endl;

		ofm.setTrianglePixelSize (ofmPixelSize);
		ofm.setInputCloud(inputCloud_);
		ofm.reconstruct (*triangles_);
		std::cout << "Reconstructed triangles: " << triangles_->size() << std::endl;

		// make an ofMesh
		outputMesh_.clear();
		PointType p;

		// triangle mesh
		outputMesh_.setMode(OF_PRIMITIVE_TRIANGLES);
		// the old "crap":
		// -----> for(std::vector<pcl::Vertices>::iterator it = temp_verts->begin(); it != temp_verts->end(); ++it) {
		// The NEW shiny C++11 style :))) 
		for(auto &t : *triangles_) {
			// So easy, such style, very beauty, many readable, so wow!
			for(auto &pointindex : t.vertices){
				p = inputCloud_->at(pointindex);
				outputMesh_.addVertex(ofVec3f(-p.x*1000,-p.y*1000,p.z*1000));
				outputMesh_.addColor(ofColor(p.r,p.g,p.b));
				//TODO: add normals, texturecoordinates
			}
		}
		std::cout << "Mesh Size after meshing: " << outputMesh_.getNumVertices() << std::endl;

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

