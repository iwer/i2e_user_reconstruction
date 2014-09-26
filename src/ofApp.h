#pragma once

#include "ofMain.h"
#include "ofxUi.h"
#include "Controls.h"
#include "AbstractProcessingPipeline.h"
#include "AbstractPointCloudGenerator.h"

#include "PclOpenNI2Grabber.h"
#include "Pipeline01.h"

#include <pcl/common/time.h> //fps calculations
#include <pcl/common/angles.h>
#include <pcl/io/openni2_grabber.h>


#include "typedefs.h"

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
	do \
{ \
	static unsigned count = 0;\
	static double last = pcl::getTime ();\
	double now = pcl::getTime (); \
	++count; \
	if (now - last >= 1.0) \
{ \
	std::cout << "Average framerate ("<< _WHAT_ << "): " << double (count)/double (now - last) << " Hz" <<  std::endl; \
	count = 0; \
	last = now; \
} \
}while (false)
#else
#define FPS_CALC (_WHAT_) \
	do \
{ \
}while (false)
#endif

#define RENDER_POINTS 0
#define RENDER_WIRE 1
#define RENDER_MESH 2

class ofApp : public ofBaseApp{
public:
	ofApp();
	~ofApp();

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

	ofEasyCam cam_;

	/*
	CloudConstPtr temp_cloud_;

	Controls * control_;

	AbstractProcessingPipeline * pipeline_;
	AbstractPointCloudGenerator * cloudSource_;
	*/
};
