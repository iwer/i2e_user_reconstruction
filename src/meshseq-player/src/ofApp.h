#pragma once

#include "ofMain.h"
#include <ofxGui.h>

class ofApp : public ofBaseApp{

	public:
		struct Playsettings
		{
			int take;
			int speed;
			int quality;
			bool texmap;
		};

		void setupUI();
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
	
		// Button Listeners
		void nextTake();
		void prevTake();
		void nextSpeed();
		void prevSpeed();
		void nextQuality();
		void prevQuality();
		void invokeLoader();
		
		int count_files(std::string basepath);
		
		void loadRandomPlaySettingData();
		void updateMaxFrames();

		float loadState_;

		Playsettings randomPlaysettings();
		Playsettings currentSettings_;
		std::vector<Playsettings> availablePlaysettings_;

		ofMesh mesh_;
		ofImage image_;

		ofEasyCam cam_;

		int frameNum_;
		int maxFrames_;

		std::mutex ressourceLoaderMtx_;
		std::thread * loaderthread_;
		std::atomic<bool> loading_;

		std::vector<std::string> take_;
		std::vector<std::string> speed_;
		std::vector<std::string> quality_;

		// meshes_[takeIdx_][speedIdx_][qualityIdx_][frameNum_]
		std::map<int, ofMesh> meshes_;
		std::map<int, ofImage> images_;

		ofTrueTypeFont font;

		// UI
		ofxPanel ui_;
		ofxButton randomRecordingBtn_;
		ofxToggle play_;

		ofPath loadingCircle_;
};
