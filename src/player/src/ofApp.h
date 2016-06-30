#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "common/PointCloudPlayer.h"
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
		ofxButton loadCalibrationBtn_;

		std::map<int, PointCloudPlayer::Ptr> player_;
		std::map<int, int> frameNumber_;
		std::map<int, recon::CloudPtr> cloud_;
		std::map<int, ofMesh> mesh_;
		std::map<int, std::shared_ptr<ofImage>> image_;
		std::list<recon::AbstractSensor::Ptr> sensors_;

		std::mutex mapLock_;
		void cloudCallback(int frameNumber, int sensorIndex, recon::CloudPtr cloud, std::shared_ptr<ofImage> image);
		
		void updateFps(int &fps);
		void changeSensor(int &sensor);

		std::vector<ofRectangle> imageLayout_;

};
