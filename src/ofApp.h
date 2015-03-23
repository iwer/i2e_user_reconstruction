#pragma once

#include "ofMain.h"
#include "ofxUi.h"
#include "Controls.h"
#include "recon/AbstractProcessingPipeline.h"
#include "recon/AbstractPointCloudGenerator.h"

#include "recon/PclOpenNI2Grabber.h"
#include "recon/DepthFilePointCloudGenerator.h"
#include "recon/Pipeline01.h"
#include "recon/Pipeline02.h"
#include "ofxMSATimer.h"
#include "ofxTimeMeasurements.h"


#include <pcl/common/time.h> //fps calculations
#include <pcl/common/angles.h>
#include <pcl/io/openni2_grabber.h>


#include "recon/typedefs.h"

#define RENDER_POINTS 0
#define RENDER_WIRE 1
#define RENDER_MESH 2

#define USE_MSA_TIMER

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

	void createOfMesh(CloudConstPtr inputCloud, TrianglesPtr triangles);
	void createOfMesh(CloudConstPtr inputCloud);
private:
	ofEasyCam cam_;
		
	//CloudConstPtr temp_cloud_;

	AbstractProcessingPipeline * pipeline_;
	AbstractPointCloudGenerator * cloudSource_;
	
	ofMesh mesh;

	float background;
	int rendermode;

	CloudConstPtr tmpCloud;
};
