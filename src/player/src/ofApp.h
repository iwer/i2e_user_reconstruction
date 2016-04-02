#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "PointCloudPlayer.h"
#include "recon/typedefs.h"
#include "of-pcl-bridge/of-pcl-bridge.h"

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
		
		ofEasyCam cam_;
		PointCloudPlayer player_;

		ofxPanel ui_;
		ofxIntSlider fpsSlider_;
		
		int frameNumber_;
		recon::CloudPtr cloud_;
		ofMesh mesh;

		void cloudCallback(int frameNumber, recon::CloudPtr cloud);
		void updateFps(int &fps);

};
