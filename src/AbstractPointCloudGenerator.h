#pragma once

#include "ofMain.h"
#include "ofxUi.h"
#include "Controls.h"
#include "typedefs.h"

class AbstractPointCloudGenerator {
public:
	AbstractPointCloudGenerator(void);
	~AbstractPointCloudGenerator(void);

	virtual void aquireFrame() = 0;
	ConstCloudPtr getOutputCloud();

	virtual void guiEvent(ofxUIEventArgs &e) = 0;

protected:
	boost::mutex cloud_mutex_;

	CloudConstPtr cloud_;
};

