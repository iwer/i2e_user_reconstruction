#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	//TIME_SAMPLE_SET_FRAMERATE( 30.0f ); //set the app's target framerate (MANDATORY)
	//TIME_SAMPLE_SET_DRAW_LOCATION( TIME_MEASUREMENTS_TOP_RIGHT );

	recon::SensorFactory s;
	s.checkConnectedDevices();

	ofEnableDepthTest();

	// connect control callbacks
	std::cout << "Connecting App Callbacks" << std::endl;
	Controls::getInstance().updateBackground.connect(boost::bind(&ofApp::setBackground, this, _1));
	Controls::getInstance().updateRenderMode.connect(boost::bind(&ofApp::setRendermode, this, _1));

	// create pipeline with control callbacks
	std::cout << "Creating Pipeline" << std::endl;
	pipeline_ = new recon::Pipeline02(NCLOUDS, 
		&Controls::getInstance().updateMinDepth,
		&Controls::getInstance().updateMaxDepth,
		&Controls::getInstance().updateTriangleSize,
		&Controls::getInstance().updateNormalKNeighbour,
		&Controls::getInstance().updateMu,
		&Controls::getInstance().updateMaxNearestNeighbours,
		&Controls::getInstance().updateSampleResolution);

	//setup grabbers
	std::cout << "Create Pointcloud sources" << std::endl;

	std::string filenames[4] = {
		"data/scan01.pcd",
		"data/scan02.pcd",
		"data/scan03.pcd",
		"data/scan04.pcd"
	};

	std::string bgFilenames[4] = {
		"data/background01.pcd",
		"data/background02.pcd",
		"data/background03.pcd",
		"data/background04.pcd"
	};

	cloudColors[0].set(255,0,0);
	cloudColors[1].set(0,255,0);
	cloudColors[2].set(0,0,255);
	cloudColors[3].set(255,255,0);

	for(auto i = 0; i < NCLOUDS; i++) {
		//	std::cout << "Loading " << filenames[i] << std::endl;
		//	cloudSource_[i] = new recon::FilePointCloudGenerator(filenames[i]);
		//	cloudSource_[i]->start();
		sensors_[i] = s.createFilePointCloudGenerator(filenames[i], bgFilenames[i]);
		sensors_[i]->setBackground();
		pipeline_->setSensor(sensors_[i], i);

		auto temp_cloud_ = sensors_[i]->getCloudSource()->getOutputCloud();
		if(temp_cloud_ && temp_cloud_->size() > 0){
			createIndexedOfMesh(temp_cloud_, i, inputMesh[i]);
		}
	}



	// setup camera
	std::cout << "Setup camera" << std::endl;
	cam_.setPosition(ofVec3f(0, 0, 0));
	cam_.setFov(57);
	cam_.lookAt(ofVec3f(0, 0, 4000), ofVec3f(0, 1, 0));
	cam_.setNearClip(.1);
	cam_.setFarClip(100000000);

	Controls::getInstance().loadSettings();
}

//--------------------------------------------------------------
void ofApp::update(){
	// Update Framerate in Gui
	Controls::getInstance().updateFramerate(ofGetFrameRate());
	// See if we can get a cloud. If we cant get one because Grabber is writing, we render the last frame again.



	//for(auto i = 0; i < NCLOUDS; i++){
	//	if(sensors_[i]){
	//		auto temp_cloud_ = sensors_[i]->getCloudSource()->getOutputCloud();
	//		if(temp_cloud_ && temp_cloud_->size() > 0){
	//			createIndexedOfMesh(temp_cloud_, i, inputMesh[i]);
	//		}
	//	}
	//}

	pipeline_->processData();
	createOfMeshFromPointsAndTriangles(pipeline_->getOutputCloud(), pipeline_->getTriangles(), outputMesh);
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(background);

	cam_.begin();
	ofPushMatrix();
	//TS_START("drawing");
	ofDrawAxis(1000);
	//ofDrawGridPlane(1000);
	//std::cout << "Mesh vertices count: " << inputMesh->getNumVertices() << std::endl;
	switch(rendermode){
	case RENDER_SOURCES:
		for(auto &m : inputMesh) {
			m.drawVertices();
		}
		break;
	case RENDER_POINTS:
		outputMesh.drawVertices();
		break;
	case RENDER_WIRE:
		outputMesh.drawWireframe();
		break;
	case RENDER_MESH:
		outputMesh.draw();
		break;
	default:
		outputMesh.drawVertices();
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
	Controls::getInstance().saveSettings();
	//for (auto &cs : cloudSource_) {
	//	cs->stop();
	//}

}

//--------------------------------------------------------------
void ofApp::setBackground(float color){
	background = color;
}

//--------------------------------------------------------------
void ofApp::setRendermode(int mode){
	rendermode = mode;
}

void ofApp::createOfMeshFromPointsAndTriangles(recon::CloudConstPtr inputCloud, recon::TrianglesPtr triangles, ofMesh &targetMesh)
{

	// triangle inputMesh
	targetMesh.clear();
	targetMesh.setMode(OF_PRIMITIVE_TRIANGLES);
	recon::PointType p;
	for(auto &t : *triangles) {
		// So easy, such style, very beauty, many readable, so wow!
		for(auto &pointindex : t.vertices){
			p = inputCloud->at(pointindex);
			targetMesh.addVertex(ofVec3f(-p.x*1000,-p.y*1000,p.z*1000));
			targetMesh.addColor(ofColor(p.r,p.g,p.b));
			//TODO: add normals, texturecoordinates
		}
	}
	//std::cout << "Mesh Size after meshing: " << outputMesh_.getNumVertices() << std::endl;

}

void ofApp::createOfMeshFromPoints(recon::CloudConstPtr inputCloud, ofMesh &targetMesh)
{

	// triangle inputMesh
	targetMesh.clear();
	targetMesh.setMode(OF_PRIMITIVE_POINTS);
	for(auto &p : inputCloud->points){
		targetMesh.addVertex(ofVec3f(-p.x*1000,-p.y*1000,p.z*1000));
		targetMesh.addColor(ofColor(p.r,p.g,p.b));
		//targetMesh.addColor(cloudColors[meshIndex]);
	}
	std::cout << "Mesh Size after meshing: " << targetMesh.getNumVertices() << " " << inputCloud->size() << std::endl;
}

void ofApp::createIndexedOfMesh(recon::CloudConstPtr inputCloud, int meshIndex, ofMesh &targetMesh)
{

	// triangle inputMesh
	targetMesh.clear();
	targetMesh.setMode(OF_PRIMITIVE_POINTS);
	for(auto &p : inputCloud->points){
		targetMesh.addVertex(ofVec3f(-p.x*1000,-p.y*1000,p.z*1000));
		//inputMesh[meshIndex].addColor(ofColor(p.r,p.g,p.b));
		targetMesh.addColor(cloudColors[meshIndex]);
	}
}
