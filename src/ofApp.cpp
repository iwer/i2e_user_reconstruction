#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup2(){
	this->splashScreen.init("splash.png");
	this->splashScreen.begin();

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
		"data/vpscan01.pcd",
		"data/vpscan02.pcd",
		"data/vpscan03.pcd",
		"data/vpscan04.pcd"
	};

	std::string bgFilenames[4] = {
		"data/vpbackground01.pcd",
		"data/vpbackground02.pcd",
		"data/vpbackground03.pcd",
		"data/vpbackground04.pcd"
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

		auto ext = sensors_[i]->getDepthExtrinsics();
		sourceTranslation[i].set(ext->getTranslation().x(), ext->getTranslation().y(), ext->getTranslation().z());
		sourceRotation[i].set(ext->getRotation().x(), ext->getRotation().y(), ext->getRotation().z(), ext->getRotation().w());


		auto temp_cloud_ = sensors_[i]->getCloudSource()->getOutputCloud();
		if(temp_cloud_ && temp_cloud_->size() > 0){
			createIndexedOfMesh(temp_cloud_, i, inputMesh[i]);
		}
	}



	// setup camera
	std::cout << "Setup camera" << std::endl;
	cam_.setPosition(ofVec3f(0, 0, 0));
	cam_.setFov(57);
	cam_.lookAt(ofVec3f(0, 4000, 0), ofVec3f(0, 0, 1));
	cam_.setNearClip(.1);
	cam_.setFarClip(100000000);

	Controls::getInstance().loadSettings();
	//ofSetFullscreen(true);
	this->splashScreen.end();
	fullyInitialized = true;
}

//--------------------------------------------------------------
void ofApp::update(){
	if (ofGetFrameNum() == 1)
	{
		nextframe = false;
		this->setup2();
	}
	else if (fullyInitialized) 
	{
		// Update Framerate in Gui
		Controls::getInstance().updateFramerate(ofGetFrameRate());
		pipeline_->processData();

		//createOfMeshFromPointsAndTriangles(pipeline_->getOutputCloud(), pipeline_->getTriangles(), outputMesh);
		createOfMeshFromPoints(pipeline_->getOutputCloud(), outputMesh);
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
	//std::cout << "Mesh vertices count: " << inputMesh->getNumVertices() << std::endl;
	switch(rendermode){
	case RENDER_SOURCES:
		for(auto i = 0; i < NCLOUDS; i++) {
			ofPushMatrix();
			if(Controls::getInstance().transformSources) {

				ofTranslate(-sourceTranslation[i].x * 1000, -sourceTranslation[i].y * 1000, sourceTranslation[i].z * 1000);

				ofVec3f qaxis; float qangle;
				sourceRotation[i].getRotate(qangle, qaxis);
				ofRotate(qangle, -qaxis.x , -qaxis.y, qaxis.z);
			}
			inputMesh[i].drawVertices();
			ofDrawAxis(100);
			ofPopMatrix();
		}
		break;
	case RENDER_POINTS:
		ofPushMatrix();
		outputMesh.drawVertices();
		ofPopMatrix();
		break;
	case RENDER_WIRE:
		ofPushMatrix();
		outputMesh.drawWireframe();
		ofPopMatrix();
		break;
	case RENDER_MESH:
		ofPushMatrix();
		outputMesh.draw();
		ofPopMatrix();
		break;
	default:
		ofPushMatrix();
		outputMesh.drawVertices();
		ofPopMatrix();
		break;
	}
	//TS_STOP("drawing");

	ofPopMatrix();
	cam_.end();

}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if(key == ' ')
	{
		nextframe = true;
	}
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
	//std::cout << "Mesh Size after meshing: " << targetMesh.getNumVertices() << " " << inputCloud->size() << std::endl;
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
