#pragma once

#include "ofMain.h"
#include <recon/AbstractSensor.h>
#include <ofxGui.h>

class ofApp : public ofBaseApp{

	public:
		ofApp()
			: resolution_("Resolution", .03, .005, .1)
			, passMin_("Z Min", .01, .01, 8)
			, passMax_("Z Max", 2.5, .01, 8)
			, background_("Background", 127, 0, 255)
			, triEdgeLength_("Triangle Edge Length", 5, 1, 50)
			, angleTolerance_("Angle Tolerance", 15, 1, 180)
			, distanceTolerance_("Distance Tolerance", .2, .001, .5)
		{}
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

		int selected_sensor_;

		ofEasyCam cam_;

		std::list<recon::AbstractSensor::Ptr> sensor_list_;
		std::map<int, ofTexture> sensor_images_;
		std::map<int, ofMesh> sensor_meshes_;


		// gui
		ofParameter<float> background_;
		ofParameter<float> passMin_;
		ofParameter<float> passMax_;
		ofParameter<float> resolution_;
		ofParameter<int> triEdgeLength_;
		ofParameter<float> angleTolerance_;
		ofParameter<float> distanceTolerance_;

		ofxFloatSlider backgroundSl_;
		ofxFloatSlider passMinSl_;
		ofxFloatSlider passMaxSl_;
		ofxFloatSlider resolutionSl_;
		ofxIntSlider triEdgeLengthSl_;
		ofxFloatSlider angleToleranceSl_;
		ofxFloatSlider distanceToleranceSl_;
		ofxButton loadCalibrationBtn_;
		ofxToggle fillWireFrameTgl_;

		ofxPanel ui_;

};
