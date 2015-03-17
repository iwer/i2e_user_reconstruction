#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	//TIME_SAMPLE_SET_FRAMERATE( 30.0f ); //set the app's target framerate (MANDATORY)
	//TIME_SAMPLE_SET_DRAW_LOCATION( TIME_MEASUREMENTS_TOP_RIGHT );
	
	ofEnableDepthTest();

	// connect control callbacks
	std::cout << "Connecting App Callbacks" << std::endl;
	Controls::getInstance().updateBackground.connect(boost::bind(&ofApp::setBackground, this, _1));
	Controls::getInstance().updateRenderMode.connect(boost::bind(&ofApp::setRendermode, this, _1));
	
	// create pipeline with control callbacks
	std::cout << "Creating Pipeline" << std::endl;
	pipeline_ = new Pipeline01(&Controls::getInstance().updateMinDepth,
		&Controls::getInstance().updateMaxDepth,
		&Controls::getInstance().updateTriangleSize);

	//setup grabbers
	std::cout << "Create Pointcloud sources" << std::endl;
	cloudSource_ = new PclOpenNI2Grabber();
	cloudSource_->start();

	// setup camera
	std::cout << "Setup camera" << std::endl;
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
		createOfMesh(pipeline_->getInputCloud(), pipeline_->getTriangles());
		
		//createOfMesh(temp_cloud_);
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(background);

	cam_.begin();
	ofPushMatrix();
   	//TS_START("drawing");
	
	//std::cout << "Mesh vertices count: " << mesh->getNumVertices() << std::endl;
	switch(rendermode){
	case RENDER_POINTS:
		mesh.drawVertices();
		break;
	case RENDER_WIRE:
		mesh.drawWireframe();
		break;
	case RENDER_MESH:
		mesh.draw();
		break;
	default:
		mesh.drawVertices();
		break;
	}
   	//TS_STOP("drawing");

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

void ofApp::createOfMesh(CloudConstPtr inputCloud, TrianglesPtr triangles)
{

	// triangle mesh
	mesh.clear();
	mesh.setMode(OF_PRIMITIVE_TRIANGLES);
	PointType p;
	for(auto &t : *triangles) {
		// So easy, such style, very beauty, many readable, so wow!
		for(auto &pointindex : t.vertices){
			p = inputCloud->at(pointindex);
			mesh.addVertex(ofVec3f(-p.x*1000,-p.y*1000,p.z*1000));
			mesh.addColor(ofColor(p.r,p.g,p.b));
			//TODO: add normals, texturecoordinates
		}
	}
	//std::cout << "Mesh Size after meshing: " << outputMesh_.getNumVertices() << std::endl;

}

void ofApp::createOfMesh(CloudConstPtr inputCloud)
{

	// triangle mesh
	mesh.clear();
	mesh.setMode(OF_PRIMITIVE_POINTS);
	for(auto &p : inputCloud->points){
		mesh.addVertex(ofVec3f(-p.x*1000,-p.y*1000,p.z*1000));
		mesh.addColor(ofColor(p.r,p.g,p.b));
	}
}
