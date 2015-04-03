#pragma once

#include "ofMain.h"
#include "ofxUi.h"
#include "Controls.h"
#include "recon/AbstractProcessingPipeline.h"
#include "recon/AbstractPointCloudGenerator.h"

#include "recon/PclOpenNI2Grabber.h"
#include "recon/DepthFilePointCloudGenerator.h"
#include "recon/FilePointCloudGenerator.h"
#include "recon/Pipeline01.h"
#include "recon/Pipeline02.h"
//#include "ofxMSATimer.h"
//#include "ofxTimeMeasurements.h"


#include <pcl/common/time.h> //fps calculations
#include <pcl/common/angles.h>
#include <pcl/io/openni2_grabber.h>


#include "recon/typedefs.h"

#define NCLOUDS 4

class ofApp : public ofBaseApp{
public:
	void setup() override;
	void update() override;
	void draw() override;

	void keyPressed(int key) override;
	void keyReleased(int key) override;
	void mouseMoved(int x, int y ) override;
	void mouseDragged(int x, int y, int button) override;
	void mousePressed(int x, int y, int button) override;
	void mouseReleased(int x, int y, int button) override;
	void windowResized(int w, int h) override;
	void dragEvent(ofDragInfo dragInfo) override;
	void gotMessage(ofMessage msg) override;
	void exit() override;

	void setBackground(float color);
	void setRendermode(int mode);

	void createOfMeshFromPointsAndTriangles(CloudConstPtr inputCloud, TrianglesPtr triangles, ofMesh &targetMesh);
	void createOfMeshFromPoints(CloudConstPtr inputCloud, ofMesh &targetMesh);
	void createIndexedOfMesh(CloudConstPtr inputCloud, int meshIndex, ofMesh &targetMesh);
private:
	ofEasyCam cam_;
		
	AbstractProcessingPipeline * pipeline_;
	AbstractPointCloudGenerator * cloudSource_[NCLOUDS];
	
	ofMesh inputMesh[NCLOUDS];
	ofMesh outputMesh;

	float background;
	int rendermode;

	CloudConstPtr tmpCloud;

	ofColor cloudColors[NCLOUDS];
};
