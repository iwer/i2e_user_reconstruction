#pragma once

#include "ofMain.h"
#include <recon/SensorFactory.h>
#include <ofxGui.h>

class ofApp : public ofBaseApp{

	public:
		ofApp();
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
		void guiUpdatedExtrinsics(float &dummy);

		void loadCalibrationFromFile();
		void saveCalibrationToFile();

		int sensorCount_;
		std::vector<int> sensorIds_;
		std::map<int, recon::AbstractSensor::Ptr> sensor_list_;

		ofEasyCam cam_;

		std::map<int, ofMesh> mesh_map_;
		std::map<int, ofTexture> image_map_;
		std::map<int, ofColor> cloudColor_;
		std::map<int, ofMatrix4x4> sensor_extrinsics_;
		int selected_sensor_id_;


		ofxPanel ui_;
		ofxButton nextCamBtm_;
		ofxButton prevCamBtm_;

		ofxFloatSlider xTransSl_;
		ofxFloatSlider yTransSl_;
		ofxFloatSlider zTransSl_;
		ofxFloatSlider xRotSl_;
		ofxFloatSlider yRotSl_;
		ofxFloatSlider zRotSl_;
		ofxFloatSlider passMinSl_;
		ofxFloatSlider passMaxSl_;
		ofxIntSlider triEdgeLengthSl_;
		ofxFloatSlider angleToleranceSl_;
		ofxFloatSlider distanceToleranceSl_;
		ofxButton saveCalibrationBtn_;
		ofxButton loadCalibrationBtn_;

		ofParameter<float> xTrans_;
		ofParameter<float> yTrans_;
		ofParameter<float> zTrans_;
		ofParameter<float> xRot_;
		ofParameter<float> yRot_;
		ofParameter<float> zRot_;
		ofParameter<float> passMin_;
		ofParameter<float> passMax_;
		ofParameter<int> triEdgeLength_;
		ofParameter<float> angleTolerance_;
		ofParameter<float> distanceTolerance_;
};
