#pragma once

#include "ofMain.h"
#include "ofxUi.h"
#include "Controls.h"
#include "typedefs.h"

class AbstractPointProcessor
{
public:
	AbstractPointProcessor(void);
	~AbstractPointProcessor(void);

	void setInputCloud(CloudConstPtr);
	virtual void processData() = 0;
	CloudPtr getOutputCloud();

	virtual void guiEvent(ofxUIEventArgs &e) = 0;

protected:
	CloudConstPtr inputCloud_;
	CloudPtr outputCloud_;
};

