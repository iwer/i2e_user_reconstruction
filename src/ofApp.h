#pragma once

#include "ofMain.h"
#include "ofxUi.h"
#include "Controls.h"
#include "AbstractProcessingPipeline.h"
#include "AbstractPointCloudGenerator.h"

#include "PclOpenNI2Grabber.h"
#include "Pipeline01.h"
#include "ofxMSATimer.h"
#include "ofxTimeMeasurements.h"


#include <pcl/common/time.h> //fps calculations
#include <pcl/common/angles.h>
#include <pcl/io/openni2_grabber.h>


#include "typedefs.h"

#define RENDER_POINTS 0
#define RENDER_WIRE 1
#define RENDER_MESH 2

class ofApp : public ofBaseApp{
public:
	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y );
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);
	void exit();

	void setBackground(float color);
	void setRendermode(int mode);

private:
	ofEasyCam cam_;
		
	CloudConstPtr temp_cloud_;

	AbstractProcessingPipeline * pipeline_;
	AbstractPointCloudGenerator * cloudSource_;
	
	float background;
	int rendermode;
};
