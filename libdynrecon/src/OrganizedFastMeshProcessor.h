#pragma once
#include "AbstractMeshProcessor.h"
#include <pcl/surface/organized_fast_mesh.h>

#include "typedefs.h"

/**
  * Very fast Triangulator. Relies on organized Point Clouds because it exploits the known neighbourhood relations.
  */
class OrganizedFastMeshProcessor :
	public AbstractMeshProcessor
{
public:
	OrganizedFastMeshProcessor(void);
	~OrganizedFastMeshProcessor(void);

	void processData();

	int getEdgeLength();
	void setEdgeLength(int value); 
private:
	pcl::OrganizedFastMesh<PointType> ofm;

	int ofmPixelSize;


};

