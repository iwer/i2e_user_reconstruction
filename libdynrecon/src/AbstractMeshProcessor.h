#pragma once
#include "ofMain.h"
#include "ofxUi.h"
#include "typedefs.h"

class AbstractMeshProcessor
{
public:
	AbstractMeshProcessor(void);
	~AbstractMeshProcessor(void);

	void setInputCloud(CloudConstPtr);
	virtual void processData() = 0;
	ofMesh * getOutputMesh();
	virtual void guiEvent(ofxUIEventArgs &e) = 0;
protected:
	CloudConstPtr inputCloud_;
	ofMesh outputMesh_;
};

