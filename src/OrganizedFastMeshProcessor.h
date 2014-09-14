#pragma once
#include "AbstractMeshProcessor.h"
#include "Controls.h"
#include <pcl/surface/organized_fast_mesh.h>

#include "typedefs.h"

class OrganizedFastMeshProcessor :
	public AbstractMeshProcessor
{
public:
	OrganizedFastMeshProcessor(void);
	~OrganizedFastMeshProcessor(void);

	void processData();
	void guiEvent(ofxUIEventArgs &e);
private:
	pcl::OrganizedFastMesh<PointType> ofm;
	boost::shared_ptr<std::vector<pcl::Vertices> > vertices_;

	int ofmPixelSize;


};

