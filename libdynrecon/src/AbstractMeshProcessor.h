#pragma once
#include "ofMain.h"
#include "typedefs.h"

class AbstractMeshProcessor
{
public:
	AbstractMeshProcessor(void);
	~AbstractMeshProcessor(void);

	void setInputCloud(CloudConstPtr);
	virtual void processData() = 0;
	ofMesh * getOutputMesh();
protected:
	CloudConstPtr inputCloud_;
	ofMesh outputMesh_;
};

