#pragma once

#include "ofMain.h"
#include <ofxGui.h>
#include <recon/typedefs.h>
#include <recon/AbstractSensor.h>
#include <common/PointCloudPlayer.h>
#include <common/ModelWriter.h>
#include <ServerAPI.h>

class ofApp : public ofBaseApp{

	public:
		ofApp()
			: background_("Backgroundcolor", 127, 0, 255)
			, fps_("FPS", 1, 0, 30)
			, passMin_("Z Min", .01, .01, 80)
			, passMax_("Z Max", 3, .01, 80)
			, triEdgeLength_("Triangle Edge Length", 25, 1, 50)
			, angleTolerance_("Angle Tolerance", 15, 1, 270)
			, distanceTolerance_("Distance Tolerance", .2, .001, 10)
			, playing_("Play", false)
		{}

		void setupUi();

		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		void loadCalibrationFromFile();
		void resetCalibration();
		void cloudCallback(int frameNumber, int sensorIndex, recon::CloudPtr cloud, std::shared_ptr<ofImage> image);
		void updateFps(int &fps);
		void play(bool &play);
		void stop();
		void back();
		void nextFrame();
		void prevFrame();
		void saveCurrentFrame();

		void processFrame();
		void processFrameTriggerInt(int &value);
		void processFrameTriggerFloat(float &value);


		int globalFrameNumber_;
		int maxFrames_;

		std::map<int,int> cloudsize;
		std::map<int, float*> clouddata;

		std::map<int,int> imagesize;
		std::map<int,char*> imagedata;

		ofFbo combinedTexture_;

		// NetKinect
		ServerAPI netkinect_api_;
		//std::map<int, PointCloudPlayer::Ptr> player_;
		//std::map<int, int> frameNumber_;
		std::map<int, recon::CloudPtr> cloud_;
		std::map<int, ofMesh> mesh_;
		std::map<int, std::shared_ptr<ofImage>> image_;
		std::list<recon::AbstractSensor::Ptr> sensors_;
		std::map<int, recon::AbstractSensor::Ptr> sensorMap_;

		ofImage dummyTex_;
		ofMesh combinedMesh_;

		std::vector<ofRectangle> imageLayout_;
		std::string fileNumber(int number);
		int writeIndex_;

		ofEasyCam cam_;

		// UI stuff
		ofxPanel ui_;

		ofxIntSlider backgroundSl_;
		ofxButton loadCalibrationBtn_;
		ofxButton resetCalibrationBtn_;

		ofxToggle perPixelColor_;
		ofxToggle showFrustum_;
		ofxToggle showSingle_;
		ofxToggle showCombined_;
		ofxToggle fillWireFrameTgl_;
		ofxToggle onlyPointsTgl_;

		ofxIntSlider fpsSlider_;
		ofxButton backBtn_;
		ofxToggle playTgl_;
		ofxButton prevFrameBtn_;
		ofxButton nextFrameBtn_;
		ofxButton stopBtn_;
		ofxToggle reconstructAllTgl_;

		ofxButton saveCurrentFrame_;

		ofxFloatSlider passMinSl_;
		ofxFloatSlider passMaxSl_;
		ofxIntSlider triEdgeLengthSl_;
		ofxFloatSlider angleToleranceSl_;
		ofxFloatSlider distanceToleranceSl_;

		ofParameter<int> background_;
		ofParameter<int> fps_;
		ofParameter<bool> playing_;

		ofParameterGroup backgroundRemovalPrms_;
		ofParameter<float> passMin_;
		ofParameter<float> passMax_;

		ofParameterGroup triangulationPrms_;
		ofParameter<int> triEdgeLength_;
		ofParameter<float> angleTolerance_;
		ofParameter<float> distanceTolerance_;

};
