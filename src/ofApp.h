#pragma once

#include "ofMain.h"
#include "ofxUi.h"
#include "ofxMSATimer.h"
#include "ofxTimeMeasurements.h"


#include <pcl/common/time.h> //fps calculations
#include <pcl/common/angles.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/organized_fast_mesh.h>

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

	void cloud_callback (const CloudConstPtr& cloud);

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
	void guiEvent(ofxUIEventArgs &e);

	pcl::PassThrough<PointType> pass_;

	pcl::OrganizedFastMesh<PointType> ofm;
	boost::shared_ptr<std::vector<pcl::Vertices> > vertices_;

	pcl::io::OpenNI2Grabber * grabber_;
	boost::mutex cloud_mutex_;

	CloudConstPtr cloud_;
	CloudConstPtr temp_cloud;
	ofMesh mesh;
	ofEasyCam cam;

	ofxUICanvas *gui;
	ofxUIMovingGraph * fpsMovingGraph;

	int renderMode;

	float background;

	float depthThreshMax;
	float depthThreshMin;

	int ofmPixelSize;
};
