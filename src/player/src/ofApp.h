#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "PointCloudPlayer.h"
#include "recon/typedefs.h"
#include "of-pcl-bridge/of-pcl-bridge.h"
#include <recon/AbstractSensor.h>

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
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
		void loadCalibrationFromFile();

		ofEasyCam cam_;

		ofxPanel ui_;
		ofxIntSlider fpsSlider_;
		ofxIntSlider sensorIndexSlider_;
		//ofxButton saveCalibrationBtn_;
		ofxButton loadCalibrationBtn_;

		//int frameNumber_;
		//recon::CloudPtr cloud_;
		//ofMesh mesh;

		std::map<int, PointCloudPlayer::Ptr> player_;
		std::map<int, int> frameNumber_;
		std::map<int, recon::CloudPtr> cloud_;
		std::map<int, ofMesh> mesh;
		std::list<recon::AbstractSensor::Ptr> sensors_;

		void cloudCallback(int frameNumber, int sensorIndex, recon::CloudPtr cloud);
		void updateFps(int &fps);
		void changeSensor(int &sensor);
};
