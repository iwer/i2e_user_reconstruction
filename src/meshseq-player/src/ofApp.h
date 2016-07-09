#pragma once

#include "ofMain.h"
#include <ofxGui.h>

class ofApp : public ofBaseApp{

	public:
		void setupUI();
		void loadData();
	void updateMaxFrames();
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
		
		std::string fileNumber(int number);

		void nextTake();
		void prevTake();
		void nextSpeed();
		void prevSpeed();
		void nextQuality();
		void prevQuality();
		
		int count_files(std::string basepath);
		
		ofMesh mesh_;
		ofImage image_;

		ofEasyCam cam_;

		int frameNum_;
		int maxFrames_;

		int takeIdx_;
		std::vector<std::string> take_;
		int speedIdx_;
		std::vector<std::string> speed_;
		int qualityIdx_;
		std::vector<std::string> quality_;

		// meshes_[takeIdx_][speedIdx_][qualityIdx_][frameNum_]
		std::map<int, std::map<int, std::map<int, std::map<int,ofMesh>>>> meshes_;
		std::map<int, std::map<int, std::map<int, std::map<int,ofImage>>>> images_;

		// UI
		ofxPanel ui_;

		ofxLabel takeLbl_;
		ofxButton nextTakeBtn_;
		ofxButton prevTakeBtn_;

		ofxLabel speedLbl_;
		ofxButton nextSpeedBtn_;
		ofxButton prevSpeedBtn_;

		ofxLabel qualityLbl_;
		ofxButton nextQualityBtn_;
		ofxButton prevQualityBtn_;

};
