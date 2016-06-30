#pragma once

#include "ofMain.h"
#include <ofxGui.h>
#include <recon/typedefs.h>
#include <recon/AbstractSensor.h>
#include <common/PointCloudPlayer.h>

class ofApp : public ofBaseApp {

public:
	ofApp()
		: background_("Backgroundcolor", 127, 0, 255)
		, fps_("FPS", 1, 0, 30)
		, passMin_("Z Min", .01, .01, 8)
		, passMax_("Z Max", 2.5, .01, 8)
		, playing_("Play", false)
		, resolution_("Resolution", 0.03, 0.001, 0.2)
		, normalKNeighbours_("K Neighbours", 20, 3, 500)
		, triEdgeLength_("Triangle Edge Length", 0.03, 0.001, 0.2)
		, searchRadius_("Search radius", 0.035, 0.004, 0.8)
		, mu_("Radius Multiplier", 2.5, 0.1, 10)
		, maxNeighbours_("Max Neighbours", 100, 10, 500)
		, maxSurfaceAngle_("Max Surface Angle", 45, 0, 360)
		, minAngle_("Min Triangle Angle", 10, 1, 60)
		, maxAngle_("Max Triangle Angle", 120, 60, 179)
		, smoothRadius_("Search Radius", 0.03, 0.001, 0.2)
	{}

	void setupUi();

	void setup();
	void update();
	void draw();

	void keyPressed(int key);
	void keyReleased(int key);
	void mouseMoved(int x, int y);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void mouseEntered(int x, int y);
	void mouseExited(int x, int y);
	void windowResized(int w, int h);
	void dragEvent(ofDragInfo dragInfo);
	void gotMessage(ofMessage msg);

	void loadCalibrationFromFile();
	void cloudCallback(int frameNumber, int sensorIndex, recon::CloudPtr cloud, std::shared_ptr<ofImage> image);
	void updateFps(int &fps);
	void play(bool &play);
	void stop();
	void back();
	void nextFrame();
	void prevFrame();
	void saveCurrentFrame();

	void drawNormals(ofMesh &mesh, float length, bool bFaceNormals);

	int globalFrameNumber_;
	int maxFrames_;

	ofFbo combinedTexture_;

	std::map<int, PointCloudPlayer::Ptr> player_;
	std::map<int, int> frameNumber_;
	std::map<int, recon::CloudPtr> cloud_;
	std::map<int, ofMesh> mesh_;
	std::map<int, std::shared_ptr<ofImage>> image_;
	std::list<recon::AbstractSensor::Ptr> sensors_;
	std::map<int, recon::AbstractSensor::Ptr> sensorMap_;

	ofImage dummyTex_;
	ofMesh combinedMesh_;

	std::vector<ofRectangle> imageLayout_;

	ofEasyCam cam_;

	// UI stuff
	ofxPanel ui_;

	ofxIntSlider backgroundSl_;
	ofxButton loadCalibrationBtn_;

	ofxToggle perPixelColor_;
	ofxToggle showFrustum_;
	ofxToggle showSingle_;
	ofxToggle showCombined_;
	ofxToggle showNormals_;
	ofxToggle fillWireFrameTgl_;

	ofxIntSlider fpsSlider_;
	ofxButton backBtn_;
	ofxToggle playTgl_;
	ofxButton prevFrameBtn_;
	ofxButton nextFrameBtn_;
	ofxButton stopBtn_;

	ofxButton saveCurrentFrame_;

	ofxFloatSlider passMinSl_;
	ofxFloatSlider passMaxSl_;


	ofParameter<int> background_;
	ofParameter<int> fps_;
	ofParameter<bool> playing_;

	ofParameterGroup backgroundRemovalPrms_;
	ofParameter<float> passMin_;
	ofParameter<float> passMax_;

	ofParameterGroup normalCalcPrms_;
	ofParameter<int> normalKNeighbours_;

	ofParameterGroup downsamplingPrms_; 
	ofParameter<float> resolution_;

	ofParameterGroup smoothingPrms_;
	ofParameter<float> smoothRadius_;

	ofParameterGroup triangulationPrms_;
	ofParameter<float> triEdgeLength_;
	ofParameter<float> searchRadius_;
	ofParameter<float> mu_;
	ofParameter<int> maxNeighbours_;
	ofParameter<float> maxSurfaceAngle_;
	ofParameter<float> minAngle_;
	ofParameter<float> maxAngle_;


};
