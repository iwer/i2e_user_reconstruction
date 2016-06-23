#include "ofApp.h"
#include <recon/SensorFactory.h>
#include <common/SensorCalibrationSettings.h>

//--------------------------------------------------------------
void ofApp::setup(){
	boost::filesystem::path full_path(boost::filesystem::current_path());
	std::cout << "Current path is : " << full_path << std::endl;

	boost::filesystem::path recorder_data_path("./data/recorder");

	ui_.setup();
	fpsSlider_.addListener(this, &ofApp::updateFps);
	sensorIndexSlider_.addListener(this, &ofApp::changeSensor);

	ui_.add(fpsSlider_.setup("FPS", 1, 1, 60));
	ui_.add(sensorIndexSlider_.setup("SensorIndex", 0, 0, 2));
	loadCalibrationBtn_.addListener(this, &ofApp::loadCalibrationFromFile);
	ui_.add(loadCalibrationBtn_.setup("Load calibration"));

	int sensorCount = PointCloudPlayer::getNumberSensors(full_path.generic_string() + recorder_data_path.generic_string());
	std::cout << "Sensor count: " << sensorCount << std::endl;

	recon::SensorFactory sensorFac;

	for(int i = 0; i < sensorCount; i++)
	{
		sensors_.push_back(sensorFac.createDummySensor());
		player_[sensors_.back()->getId()] = PointCloudPlayer::Ptr(new PointCloudPlayer(full_path.generic_string() + recorder_data_path.generic_string(),
			sensors_.back()->getId(), 1));
		player_[sensors_.back()->getId()]->callback.connect(boost::bind(&ofApp::cloudCallback, this, _1, _2, _3, _4));
	}

	for (auto &s : sensors_) {
		player_[s->getId()]->start();
		image_[s->getId()] = std::make_shared<ofImage>();
	}


	switch (sensorCount) {
	case 1:
		imageLayout_.push_back(ofRectangle(0, 0, ofGetWidth(), ofGetHeight()));
	case 2:
	case 3:
	case 4:
		imageLayout_.push_back(ofRectangle(0, 0, ofGetWidth() / 2, ofGetHeight() / 2));
		imageLayout_.push_back(ofRectangle(ofGetWidth() / 2, 0, ofGetWidth() / 2, ofGetHeight() / 2));
		imageLayout_.push_back(ofRectangle(0, ofGetHeight() / 2, ofGetWidth() / 2, ofGetHeight() / 2));
		imageLayout_.push_back(ofRectangle(ofGetWidth() / 2, ofGetHeight() / 2, ofGetWidth() / 2, ofGetHeight() / 2));
	}

	ofDisableArbTex();
}

//--------------------------------------------------------------
void ofApp::update(){
	for (auto &s : sensors_) {
		if (cloud_[s->getId()]) {
			createOfMeshFromPoints(cloud_[s->getId()], mesh[s->getId()]);
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0);
	ofDrawBitmapString(std::to_string(frameNumber_[0]) + std::string("/") + std::to_string(player_[0]->getNumberFrames()), ofGetWidth() - 100, 10);
	ofEnableDepthTest();

	for (auto &s : sensors_) {
		cam_.begin();
		ofPushMatrix();
		auto ext = s->getDepthExtrinsics();
		auto translation = toOfVector3(*ext->getTranslation());
		auto rotation = toOfQuaternion(*ext->getRotation());
		ofVec3f qaxis;
		float qangle;
		rotation.getRotate(qangle, qaxis);
		ofTranslate(translation);
		ofRotate(qangle, qaxis.x, qaxis.y, qaxis.z);
		mesh[s->getId()].drawVertices();
		ofPopMatrix();
		cam_.end();

		if (image_[s->getId()]->isAllocated()) {
			ofPushMatrix();
			ofTranslate(ofGetWidth() / 4 * 3, ofGetHeight() / 4 * 3);
			ofScale(.25, .25, .25);
			
			ofImage i;
			image_[s->getId()]->getTexture().draw(imageLayout_[s->getId()]);
			ofPopMatrix();
		}
	}

	ofDisableDepthTest();
	ui_.draw();


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

void ofApp::cloudCallback(int frameNumber, int sensorIndex, recon::CloudPtr cloud, std::shared_ptr<ofImage> image)
{
	//std::lock_guard<std::mutex> lock(mapLock_);
	frameNumber_[sensorIndex] = frameNumber;
	cloud_[sensorIndex].swap(cloud);
	image_[sensorIndex].swap(image);
}

void ofApp::updateFps(int &fps)
{
	for (auto &s : sensors_) {
		player_[s->getId()]->setFramesPerSecond(fps);
	}
}

void ofApp::changeSensor(int & sensor)
{
	//player_.setSensorIndex(sensor);
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

void ofApp::loadCalibrationFromFile()
{
	SensorCalibrationSettings set;
	for (auto& s : sensors_)
	{
		set.loadCalibration(s, s->getId());
	}
}
