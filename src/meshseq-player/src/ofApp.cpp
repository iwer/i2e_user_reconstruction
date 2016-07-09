#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	mesh_.load("combined.ply");
	std::cout << "Mesh has " << (mesh_.hasColors() ? "" : "no ") << "colors" << std::endl;
	std::cout << "Mesh has " << (mesh_.hasNormals() ? "" : "no ") << "normals" << std::endl;
	std::cout << "Mesh has " << (mesh_.hasTexCoords() ? "" : "no ") << "texcoords" << std::endl;

	image_.load("combined.png");
	image_.update();

	std::cout << "Image resolution " << image_.getWidth() << "x" << image_.getHeight();


	cam_.setFarClip(100000);
	cam_.rotate(180, 0, 1, 0);
	cam_.enableMouseInput();
	cam_.disableMouseMiddleButton();
}

//--------------------------------------------------------------
void ofApp::update(){
	//std::cout << "Mesh stats: " << std::endl <<
	//	mesh_.getNumVertices() << " Vertizes" << std::endl <<
	//	mesh_.getNumColors() << "Colors" << std::endl <<
	//	mesh_.getNumTexCoords() << "TexCoords" << std::endl;


//	for (auto &texcd : mesh_.getTexCoords())
//	{
//		std::cout << texcd << std::endl;
//	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(127);
	
	image_.draw(ofGetWidth() / 4 * 3, ofGetHeight() / 4 * 3, ofGetWidth() / 4, ofGetHeight() / 4);

	cam_.begin();
	
	ofEnableDepthTest();
	ofDrawAxis(1000);
	
	
	image_.getTexture().bind();
	mesh_.disableColors();
	mesh_.enableTextures();
	mesh_.draw();
	image_.getTexture().unbind();
	
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

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
