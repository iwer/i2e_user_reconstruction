#include "ofApp.h"

ofApp::ofApp()
{
}

ofApp::~ofApp() {
}

//--------------------------------------------------------------
void ofApp::setup(){
	ofEnableDepthTest();
	//setup gui
	control_ = new Controls();
	pipeline_ = new Pipeline01();
	cloudSource_ = new PclOpenNI2Grabber();

	// setup camera
	cam_.setPosition(ofVec3f(0, 0, 0));
	cam_.lookAt(ofVec3f(0, 0, 4000), ofVec3f(0, 1, 0));
}

//--------------------------------------------------------------
void ofApp::update(){

	// See if we can get a cloud. If we cant get one because Grabber is writing, we render the last frame again.
	temp_cloud_ = cloudSource_->getOutputCloud();

	if(temp_cloud_){
		pipeline_->setInputCloud(temp_cloud_);
		pipeline_->processData();
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(control_->background);

	cam_.begin();
	ofPushMatrix();

	switch(control_->renderMode){
	case RENDER_POINTS:
		pipeline_->getOutputMesh()->drawVertices();
		break;
	case RENDER_WIRE:
		pipeline_->getOutputMesh()->drawWireframe();
		break;
	case RENDER_MESH:
		pipeline_->getOutputMesh()->draw();
		break;
	default:
		pipeline_->getOutputMesh()->drawVertices();
	}

	ofPopMatrix();
	cam_.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
	// if mouse on gui
	if(Controls::getGui()->getRect()->getMaxX() >= x && Controls::getGui()->getRect()->getMaxY() >= y){
		cam_.disableMouseInput();
	} else {
		cam_.enableMouseInput();
	}
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
void ofApp::windowResized(int w, int h){
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
}

//--------------------------------------------------------------
void ofApp::exit()
{
	Controls::getGui()->saveSettings("settings.xml");
	delete control_;
}

