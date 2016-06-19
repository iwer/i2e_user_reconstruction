#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	image_ = std::make_shared<ofImage>();
	image_local_ = std::make_shared<ofImage>();
	ofEnableDepthTest();
}

//--------------------------------------------------------------
void ofApp::update(){
	image_->load("recorder/capture_s0_00001.jpg");

	image_local_.swap(image_);
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0);
	ofPushMatrix();
	ofTranslate(ofGetWidth() / 4 * 3, ofGetHeight() / 4 * 3);
	ofScale(.25, .25, .25);
	image_local_->getTexture().draw(0, 0,ofGetWidth(),ofGetHeight());
	ofPopMatrix();

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
