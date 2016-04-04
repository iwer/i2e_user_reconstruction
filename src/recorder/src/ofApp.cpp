#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	boost::filesystem::path full_path(boost::filesystem::current_path());
	std::cout << "Current path is : " << full_path << std::endl;
	
	recon::SensorFactory sensorFac;

	auto nSensors = sensorFac.checkConnectedDevices(true);
	for (int i = 0; i < nSensors; i++) {
		//sensor_list_.push_back(sensorFac.createFilePointCloudGenerator(full_path.generic_string() + std::string("/data/vpscan01.pcd"), 
		//	full_path.generic_string() + std::string("/data/vpbackground01.pcd")));
		sensor_list_.push_back(sensorFac.createPclOpenNI2Grabber());
		writer_list_.insert(std::pair<int, PointCloudWriter*>(sensor_list_.back()->getId(), new PointCloudWriter()));
		writer_list_[sensor_list_.back()->getId()]->setBaseFileName(std::string("./data/recorder/capture"));
		writer_list_[sensor_list_.back()->getId()]->setSensorDetails(sensor_list_.back());
		writer_list_[sensor_list_.back()->getId()]->start();
	}

	switch (nSensors) {
	case 1:
		imageLayout_.push_back(ofRectangle(0, 0, ofGetWidth(), ofGetHeight()));
	case 2:
	case 3:
	case 4:
		imageLayout_.push_back(ofRectangle(0, 0, ofGetWidth()/2, ofGetHeight()/2));
		imageLayout_.push_back(ofRectangle(ofGetWidth() / 2, 0, ofGetWidth() / 2, ofGetHeight() / 2));
		imageLayout_.push_back(ofRectangle(0, ofGetHeight() / 2, ofGetWidth() / 2, ofGetHeight() / 2));
		imageLayout_.push_back(ofRectangle(ofGetWidth() / 2, ofGetHeight() / 2, ofGetWidth() / 2, ofGetHeight() / 2));
	}
	
	ofSetFrameRate(30);
}

//--------------------------------------------------------------
void ofApp::update(){
	sensor_images_.clear();
	for (auto &sensor : sensor_list_) {
		ofTexture t;
		auto image = sensor->getCloudSource()->getOutputImage();
		if (image != nullptr) {
			toOfTexture(image, t);
			sensor_images_.erase(sensor->getId());
			sensor_images_.insert(std::pair<int, ofTexture>(sensor->getId(), t));
		}
		else {
			std::cerr << "Got no image from sensor " << sensor->getId() << std::endl;
		}
		auto cloud = sensor->getCloudSource()->getOutputCloud();
		if (cloud != nullptr) {
			writer_list_[sensor->getId()]->enquePointcloudForWriting(cloud);
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	for (auto &sensor : sensor_list_) {
		sensor_images_[sensor->getId()].draw(imageLayout_[sensor->getId()]);
		ofDrawBitmapString(std::string("Write Queue Length: ") + std::to_string(writer_list_[sensor->getId()]->getQueueLength()), 10, 10);
	}
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
