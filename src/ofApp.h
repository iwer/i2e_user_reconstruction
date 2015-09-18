#pragma once

#include "ofMain.h"
#include "ofxUi.h"
#include "Controls.h"
#include "recon/AbstractProcessingPipeline.h"
#include "recon/AbstractPointCloudGenerator.h"

#include "recon/SensorFactory.h"
#include "recon/DepthFilePointCloudGenerator.h"
#include "recon/FilePointCloudGenerator.h"
#include "recon/Pipeline01.h"
#include "recon/Pipeline02.h"

#include "ofxSplashScreen.h"
#include "SensorCalibrationSettings.h"

#include <pcl/common/time.h> //fps calculations
#include <pcl/common/angles.h>
#include <pcl/io/openni2_grabber.h>


#include "recon/typedefs.h"

#define NCLOUDS 1

class ofApp : public ofBaseApp{
public:
	ofApp()
	{
		fullyInitialized = false;
	}
	void setup2();

	void update() override;

	void drawReconstruction();
	void drawCalibration();
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
	void setAppmode(int mode);

	ofVec2f* calculateTextureCoordinate(ofVec3f &point, int cam_index);
	void createOfMeshFromPointsAndTriangles(recon::CloudConstPtr inputCloud, recon::TrianglesPtr triangles, ofMesh &targetMesh);
	void createOfMeshFromPoints(recon::CloudConstPtr inputCloud, ofMesh &targetMesh);
	void createIndexedOfMesh(recon::CloudConstPtr inputCloud, int meshIndex, ofMesh &targetMesh);

	void toOfTexture(recon::ImagePtr image);

	void updateCameraTransformation(float xPos,float yPos,float zPos,float xRot,float yRot, float zRot);
	void saveExtrinsicsToCurrentSensor();
	void loadExtrinsicsFromCurrentSensor();
	void selectNextCamera();

	void setTexturesEnabled(bool state);
private:
	ofEasyCam cam_;

	recon::AbstractProcessingPipeline * pipeline_;
	recon::AbstractPointCloudGenerator * cloudSource_[NCLOUDS];
	recon::AbstractSensor::Ptr sensors_[NCLOUDS];
	
	ofMesh inputMesh[NCLOUDS];
	ofColor cloudColors[NCLOUDS];
	ofPoint sourceTranslation[NCLOUDS];
	ofQuaternion sourceRotation[NCLOUDS];
	
	ofMesh outputMesh;

	float background;
	int appmode;
	int rendermode;
	int selectedCamera;

	recon::CloudConstPtr tmpCloud;

	ofxSplashScreen splashScreen;

	bool fullyInitialized;
	bool nextframe;

	ofTexture texture;
	bool textures_enabled;
};
