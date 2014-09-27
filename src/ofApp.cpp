#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	TIME_SAMPLE_SET_FRAMERATE( 30.0f ); //set the app's target framerate (MANDATORY)
	TIME_SAMPLE_SET_DRAW_LOCATION( TIME_MEASUREMENTS_TOP_RIGHT );
	
	ofEnableDepthTest();

	Controls::getInstance().updateBackground.connect(boost::bind(&ofApp::setBackground, this, _1));
	Controls::getInstance().updateRenderMode.connect(boost::bind(&ofApp::setRendermode, this, _1));
	pipeline_ = new Pipeline01(&Controls::getInstance().updateMinDepth,
		&Controls::getInstance().updateMaxDepth,
		&Controls::getInstance().updateTriangleSize);
	cloudSource_ = new PclOpenNI2Grabber();
	cloudSource_->start();

	// setup camera
	cam_.setPosition(ofVec3f(0, 0, 0));
	cam_.lookAt(ofVec3f(0, 0, 4000), ofVec3f(0, 1, 0));
}

//--------------------------------------------------------------
void ofApp::update(){
	// Update Framerate in Gui
	Controls::getInstance().updateFramerate(ofGetFrameRate());
	// See if we can get a cloud. If we cant get one because Grabber is writing, we render the last frame again.
	temp_cloud_ = cloudSource_->getOutputCloud();

	if(temp_cloud_){
		pipeline_->setInputCloud(temp_cloud_);
		pipeline_->processData();
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(background);

	cam_.begin();
	ofPushMatrix();
   	TS_START("drawing");

	switch(rendermode){
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
		break;
	}
   	TS_STOP("drawing");

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

	// disable cam when mouse on gui
	if(Controls::getInstance().getGui()->getRect()->getMaxX() >= x && Controls::getInstance().getGui()->getRect()->getMaxY() >= y){
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
	Controls::getInstance().getGui()->saveSettings("settings.xml");
	cloudSource_->stop();
}

//--------------------------------------------------------------
void ofApp::setBackground(float color){
	background = color;
}

//--------------------------------------------------------------
void ofApp::setRendermode(int mode){
	rendermode = mode;
}

