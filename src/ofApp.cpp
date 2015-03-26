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
	//pipeline_ = new Pipeline01(&Controls::getInstance().updateMinDepth,
	//	&Controls::getInstance().updateMaxDepth,
	//	&Controls::getInstance().updateTriangleSize);
	pipeline_ = new Pipeline02(&Controls::getInstance().updateMinDepth,
		&Controls::getInstance().updateMaxDepth,
		&Controls::getInstance().updateTriangleSize);

	//setup grabbers
	std::cout << "Create Pointcloud sources" << std::endl;
	//cloudSource_ = new PclOpenNI2Grabber();
	//DepthFilePointCloudGenerator * d = new DepthFilePointCloudGenerator(30);
	//cloudSource_ = (AbstractPointCloudGenerator*)d;
	//d->loadDepthImageFromFile("data/rgbscan_cam1.png", "data/scan_cam1_16bit_1.png");

	std::string filenames[4] = {
		"data/scan01.pcd",
		"data/scan02.pcd",
		"data/scan03.pcd",
		"data/scan04.pcd"
	};

	for(auto i = 0; i < 4; i++) {
		std::cout << "Loading " << filenames[i] << std::endl;
		cloudSource_[i] = new FilePointCloudGenerator(filenames[i]);
		cloudSource_[i]->start();
	}

	cloudColors[0].set(255,0,0);
	cloudColors[1].set(0,255,0);
	cloudColors[2].set(0,0,255);
	cloudColors[3].set(255,255,0);

	// setup camera
	std::cout << "Setup camera" << std::endl;
	cam_.setPosition(ofVec3f(0, 0, 0));
	cam_.setFov(57);
	cam_.lookAt(ofVec3f(0, 0, 4000), ofVec3f(0, 1, 0));
	cam_.setNearClip(.1);
	cam_.setFarClip(100000000);

	Controls::getInstance().getGui()->loadSettings("settings.xml");
}

//--------------------------------------------------------------
void ofApp::update(){
	// Update Framerate in Gui
	//Controls::getInstance().updateFramerate(ofGetFrameRate());
	// See if we can get a cloud. If we cant get one because Grabber is writing, we render the last frame again.
	for(auto i = 0; i < 4;i++){
		if(cloudSource_[i]){
			auto temp_cloud_ = cloudSource_[i]->getOutputCloud();
			if(temp_cloud_){
				pipeline_->setInputCloud(temp_cloud_);
				pipeline_->processData();
				createOfMesh(pipeline_->getInputCloud(), pipeline_->getTriangles(), i);

				//createOfMesh(temp_cloud_, i);
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(background);

	cam_.begin();
	ofPushMatrix();
	//TS_START("drawing");
	ofDrawAxis(1000);
	//ofDrawGridPlane(1000);
	//std::cout << "Mesh vertices count: " << mesh->getNumVertices() << std::endl;
	switch(rendermode){
	case RENDER_POINTS:
		for(auto &m : mesh) {
			m.drawVertices();
		}
		break;
	case RENDER_WIRE:
		for(auto &m : mesh) {
			m.drawWireframe();
		}
		break;
	case RENDER_MESH:
		for(auto &m : mesh) {
			m.draw();
		}
		break;
	default:
		for(auto &m : mesh) {
			m.drawVertices();
		}
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
	for (auto &cs : cloudSource_) {
		cs->stop();
	}
}

//--------------------------------------------------------------
void ofApp::setBackground(float color){
	background = color;
}

//--------------------------------------------------------------
void ofApp::setRendermode(int mode){
	rendermode = mode;
}

void ofApp::createOfMesh(CloudConstPtr inputCloud, TrianglesPtr triangles, int meshIndex)
{

	// triangle mesh
	mesh[meshIndex].clear();
	mesh[meshIndex].setMode(OF_PRIMITIVE_TRIANGLES);
	PointType p;
	for(auto &t : *triangles) {
		// So easy, such style, very beauty, many readable, so wow!
		for(auto &pointindex : t.vertices){
			p = inputCloud->at(pointindex);
			mesh[meshIndex].addVertex(ofVec3f(-p.x*1000,-p.y*1000,p.z*1000));
			//mesh[meshIndex].addColor(ofColor(p.r,p.g,p.b));
			mesh[meshIndex].addColor(cloudColors[meshIndex]);
			//TODO: add normals, texturecoordinates
		}
	}
	//std::cout << "Mesh Size after meshing: " << outputMesh_.getNumVertices() << std::endl;

}

void ofApp::createOfMesh(CloudConstPtr inputCloud, int meshIndex)
{

	// triangle mesh
	mesh[meshIndex].clear();
	mesh[meshIndex].setMode(OF_PRIMITIVE_POINTS);
	for(auto &p : inputCloud->points){
		mesh[meshIndex].addVertex(ofVec3f(-p.x*1000,-p.y*1000,p.z*1000));
		//mesh[meshIndex].addColor(ofColor(p.r,p.g,p.b));
		mesh[meshIndex].addColor(cloudColors[meshIndex]);
	}
}
