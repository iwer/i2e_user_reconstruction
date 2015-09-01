#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup2(){
	this->splashScreen.init("splash.png");
	this->splashScreen.begin();

	selectedCamera = 0;

	pcl::ScopeTime t("Setup");
	recon::SensorFactory s;
	s.checkConnectedDevices();

	ofEnableDepthTest();

	// connect control callbacks
	std::cout << "Connecting App Callbacks" << std::endl;
	Controls::getInstance().updateBackground.connect(boost::bind(&ofApp::setBackground, this, _1));
	Controls::getInstance().updateRenderMode.connect(boost::bind(&ofApp::setRendermode, this, _1));
	Controls::getInstance().updateCameraTransformation.connect(boost::bind(&ofApp::updateCameraTransformation, this, _1, _2, _3, _4, _5, _6));
	Controls::getInstance().nextCamera.connect(boost::bind(&ofApp::selectNextCamera, this));

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
		// sourceTranslation[i].set(ext->getTranslation().x(), ext->getTranslation().y(), ext->getTranslation().z());
		// sourceRotation[i].set(ext->getRotation().x(), ext->getRotation().y(), ext->getRotation().z(), ext->getRotation().w());
		sourceTranslation[i].set(0,0,0);
		sourceRotation[i].set(0,0,0,1);

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
	pcl::ScopeTime t("Update");

	if (ofGetFrameNum() == 1)
	{
		nextframe = false;
		this->setup2();
	}
	else if (fullyInitialized) 
	{
		// Update Framerate in Gui
		Controls::getInstance().updateFramerate(ofGetFrameRate());
		//pipeline_->processData();

		//createOfMeshFromPointsAndTriangles(pipeline_->getOutputCloud(), pipeline_->getTriangles(), outputMesh);
		//createOfMeshFromPoints(pipeline_->getOutputCloud(), outputMesh);
	}
}

void ofApp::drawReconstruction()
{
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


}

//--------------------------------------------------------------
void ofApp::draw(){
	pcl::ScopeTime t("Draw");

	ofBackground(background);

	cam_.begin();
	ofPushMatrix();
	ofDrawAxis(1000);
	//drawReconstruction();


	for(auto i = 0; i < NCLOUDS; i++) {
		ofPushMatrix();
		ofVec3f qaxis; float qangle;
		sourceRotation[i].getRotate(qangle, qaxis);
		if(Controls::getInstance().transformSources) {
			ofTranslate(-sourceTranslation[i].x * 1000, -sourceTranslation[i].y * 1000, sourceTranslation[i].z * 1000);
			ofRotate(qangle, -qaxis.x , -qaxis.y, qaxis.z);
		}
		inputMesh[i].drawVertices();
		ofDrawAxis(100);
		ofPopMatrix();

		// camera frustum on active cam
		if (i == selectedCamera) {
			ofPushMatrix();
			ofMatrix4x4 mat, persp;
			mat.translate(sourceTranslation[i].x * 1000, sourceTranslation[i].y * 1000, -sourceTranslation[i].z * 1000);
			mat.rotate(qangle, qaxis.x, qaxis.y, -qaxis.z);
			persp.makePerspectiveMatrix(37, 1.33, .1, 100000);
			mat.postMult(persp);
			ofMultMatrix(mat.getInverse());
			ofNoFill();
			ofDrawBox(0, 0, 0, 2.0f);
			ofPopMatrix();
		}
	}



	ofPopMatrix();
	cam_.end();}

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

	// disable cam when mouse on mainGui
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

void ofApp::updateCameraTransformation(float xPos, float yPos, float zPos, float xRot, float yRot, float zRot)
{
	sourceTranslation[selectedCamera].set(xPos, yPos, zPos);
	sourceRotation[selectedCamera].set(xRot, yRot, zRot, 0);
}

void ofApp::selectNextCamera()
{
	selectedCamera = (selectedCamera + 1) % NCLOUDS;
}