#pragma once

#include "ofMain.h"
#include <recon/SensorFactory.h>
#include <ofxGui.h>

class ofApp : public ofBaseApp{

	public:
		ofApp()
			: xTrans_("X Translation", 0, -5, 5)
			, yTrans_("Y Translation", 0, -5, 5)
			, zTrans_("Z Translation", 0, -5, 5)
			, xRot_("X Rotation", 0, -180, 180)
			, yRot_("Y Rotation", 0, -180, 180)
			, zRot_("Z Rotation", 0, -180, 180)
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
		
		void selectNextCamera();
		void selectPreviousCamera();
		void saveExtrinsicsToCurrentSensor();
		void loadExtrinsicsFromCurrentSensor();
		void guiUpdatedExtrinsics();

		std::list<recon::AbstractSensor::Ptr> sensor_list_;
		std::list<recon::AbstractSensor::Ptr>::iterator sensor_list_it_;

		ofEasyCam cam_;

		std::map<int, ofMesh> mesh_map_;
		std::map<int, ofColor> cloudColor_;
		std::map<int, ofMatrix4x4> sensor_extrinsics_;
		int selected_sensor_;


		ofxPanel ui_;
		ofxButton nextCamBtm_;
		ofxButton prevCamBtm_;

		ofxFloatSlider xTransSl_;
		ofxFloatSlider yTransSl_;
		ofxFloatSlider zTransSl_;
		ofxFloatSlider xRotSl_;
		ofxFloatSlider yRotSl_;
		ofxFloatSlider zRotSl_;

		ofParameter<float> xTrans_;
		ofParameter<float> yTrans_;
		ofParameter<float> zTrans_;
		ofParameter<float> xRot_;
		ofParameter<float> yRot_;
		ofParameter<float> zRot_;
};
