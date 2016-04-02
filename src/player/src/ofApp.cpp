#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	boost::filesystem::path full_path(boost::filesystem::current_path());
	std::cout << "Current path is : " << full_path << std::endl;

	boost::filesystem::path recorder_data_path("/data/recorder");

	ui_.setup();
	fpsSlider_.addListener(this, &ofApp::updateFps);

	ui_.add(fpsSlider_.setup("FPS", 1, 1, 60));



	player_.setBasePath(full_path.generic_string() + recorder_data_path.generic_string());
	player_.callback.connect(boost::bind(&ofApp::cloudCallback, this, _1, _2));
	player_.start();
}

//--------------------------------------------------------------
void ofApp::update(){
	if (cloud_) {
		createOfMeshFromPoints(cloud_, mesh);
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0);
	ofDrawBitmapString(std::to_string(frameNumber_) + std::string("/") + std::to_string(player_.getNumberFrames()), ofGetWidth() - 100, 10);

	ui_.draw();

	cam_.begin();
	ofSetColor(255, 255, 255);
	mesh.drawVertices();
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

void ofApp::cloudCallback(int frameNumber, recon::CloudPtr cloud)
{
	frameNumber_ = frameNumber;
	cloud_.swap(cloud);
}

void ofApp::updateFps(int &fps)
{
	player_.setFramesPerSecond(fps);
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
