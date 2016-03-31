#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	recon::SensorFactory sensorFac;

	auto nSensors = sensorFac.checkConnectedDevices(true);
	for (int i = 0; i < nSensors; i++) {
		sensor_list_.push_back(sensorFac.createPclOpenNI2Grabber());
		writer_list_.insert(std::pair<int, PointCloudWriter*>(sensor_list_.back()->getId(), new PointCloudWriter()));
		writer_list_[sensor_list_.back()->getId()]->setBaseFileName(std::string("./data/recorder/capture"));
		writer_list_[sensor_list_.back()->getId()]->setSensorDetails(sensor_list_.back());
	}
	
}

//--------------------------------------------------------------
void ofApp::update(){
	sensor_images_.clear();
	for (auto &sensor : sensor_list_) {
		ofTexture t;
		toOfTexture(sensor->getCloudSource()->getOutputImage(), t);
		sensor_images_.push_back(t);
		writer_list_[sensor->getId()]->enquePointcloudForWriting(sensor->getCloudSource()->getOutputCloud());

	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	sensor_images_.front().draw(0, 0, ofGetWidth(), ofGetHeight());
	ofDrawBitmapString(std::string("Write Queue Length: ") + std::to_string(writer_list_[0]->getQueueLength()), 10, 10);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
