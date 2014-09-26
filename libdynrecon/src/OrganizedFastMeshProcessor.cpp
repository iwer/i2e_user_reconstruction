#include "OrganizedFastMeshProcessor.h"


OrganizedFastMeshProcessor::OrganizedFastMeshProcessor(void)
{

	Controls::getGui()->addLabel("lOrgFastMesh","Meshing");
	ofxUISlider * trSl = Controls::getGui()->addSlider("TRIANGLESIZE", 1, 10, 100.0);
	trSl->setIncrement(1);
	trSl->setLabelPrecision(0);

	ofm.setTriangulationType (pcl::OrganizedFastMesh<PointType>::TRIANGLE_LEFT_CUT);
	ofmPixelSize = trSl->getScaledValue();
}


OrganizedFastMeshProcessor::~OrganizedFastMeshProcessor(void)
{
}

void OrganizedFastMeshProcessor::processData()
{
		// fast organized mesh triangulation
		ofm.setTrianglePixelSize (ofmPixelSize);
		ofm.setInputCloud(inputCloud_);
		ofm.reconstruct (*vertices_);

		// make an ofMesh
		outputMesh_.clear();
		PointType p;

		// triangle mesh
		outputMesh_.setMode(OF_PRIMITIVE_TRIANGLES);
		// the old "crap":
		// -----> for(std::vector<pcl::Vertices>::iterator it = temp_verts->begin(); it != temp_verts->end(); ++it) {
		// The NEW shiny C++11 style :))) 
		for(auto &v : *vertices_) {
			// So easy, such style, very beauty, many readable, so wow!
			for(auto &pointindex : v.vertices){
				p = inputCloud_->at(pointindex);
				outputMesh_.addVertex(ofVec3f(-p.x*1000,-p.y*1000,p.z*1000));
				outputMesh_.addColor(ofColor(p.r,p.g,p.b));
				//TODO: add normals, texturecoordinates
			}
		}
}

void OrganizedFastMeshProcessor::guiEvent(ofxUIEventArgs &e)
{
	if(e.getName() == "TRIANGLESIZE")
	{
		ofxUISlider *slider = e.getSlider();
		ofmPixelSize = (int)slider->getScaledValue();
	}
}
