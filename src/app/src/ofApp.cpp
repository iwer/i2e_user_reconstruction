#include "ofApp.h"
#include "ControlsOfxGui.h"


//--------------------------------------------------------------
void ofApp::setup(){
	//this->splashScreen.init("splash.png");
	//this->splashScreen.begin();

	ofSetFrameRate(40);
	selectedCamera = 0;
	appmode = APPMODE_RECON;
	currentFrame_ = recon::Frame::Ptr(new recon::Frame());

	pcl::ScopeTime t("Setup");
	recon::SensorFactory s;
	auto n_cams = s.checkConnectedDevices(true);

	if(n_cams < NCLOUDS)
	{
		std::cerr << "Not enough sensors" << std::endl;
		//ofExit(1);
	}

	ofEnableDepthTest();

	// connect control callbacks
	std::cout << "Connecting App Callbacks" << std::endl;
	ControlsOfxGui::getInstance().updateBackground.connect(boost::bind(&ofApp::setBackground, this, _1));
	ControlsOfxGui::getInstance().updateRenderMode.connect(boost::bind(&ofApp::setRendermode, this, _1));
	ControlsOfxGui::getInstance().updateCameraTransformation.connect(boost::bind(&ofApp::updateCameraTransformation, this, _1, _2, _3, _4, _5, _6));
	ControlsOfxGui::getInstance().nextCamera.connect(boost::bind(&ofApp::selectNextCamera, this));
	ControlsOfxGui::getInstance().updateAppMode.connect(boost::bind(&ofApp::setAppmode, this, _1));
	ControlsOfxGui::getInstance().updateFov.connect(boost::bind(&ofApp::updateFovOfCurrentCamera, this, _1));

	// create pipeline with control callbacks
	std::cout << "Creating Pipeline" << std::endl;
	//pipeline_ = new recon::Pipeline02(NCLOUDS, 
	//	&Controls::getInstance().updateMinDepth,
	//	&Controls::getInstance().updateMaxDepth,
	//	&Controls::getInstance().updateTriangleSize,
	//	&Controls::getInstance().updateNormalKNeighbour,
	//	&Controls::getInstance().updateMu,
	//	&Controls::getInstance().updateMaxNearestNeighbours,
	//	&Controls::getInstance().updateSampleResolution);
	pipeline_ = new recon::Pipeline01(
		&ControlsOfxGui::getInstance().updateMinDepth,
		&ControlsOfxGui::getInstance().updateMaxDepth,
		&ControlsOfxGui::getInstance().updateTriangleSize);

	//setup grabbers
	std::cout << "Create Pointcloud sources" << std::endl;
	boost::filesystem::path full_path(boost::filesystem::current_path());

	std::string filenames[4] = {
		full_path.generic_string() + std::string("/data/vpscan01.pcd"),
		full_path.generic_string() + std::string("/data/vpscan02.pcd"),
		full_path.generic_string() + std::string("/data/vpscan03.pcd"),
		full_path.generic_string() + std::string("/data/vpscan04.pcd")
	};

	std::string bgFilenames[4] = {
		full_path.generic_string() + std::string("/data/vpbackground01.pcd"),
		full_path.generic_string() + std::string("/data/vpbackground02.pcd"),
		full_path.generic_string() + std::string("/data/vpbackground03.pcd"),
		full_path.generic_string() + std::string("/data/vpbackground04.pcd")
	};

	cloudColors[0].set(255,0,0);
	cloudColors[1].set(0,255,0);
	cloudColors[2].set(0,0,255);
	cloudColors[3].set(255,255,0);

	SensorCalibrationSettings cal_set;

	for(auto i = 0; i < std::min(NCLOUDS, n_cams); i++) {
		std::cout << "Creating sensor" << std::endl;
		sensors_[i] = s.createFilePointCloudGenerator(filenames[i], bgFilenames[i]);
		//sensors_[i] = s.createPclOpenNI2Grabber();
		cal_set.loadCalibration(sensors_[i], i);

		pipeline_->setSensor(sensors_[i], i);

		auto ext = sensors_[i]->getDepthExtrinsics();
		sourceTranslation[i].set(ext->getTranslation()->x(), ext->getTranslation()->y(), ext->getTranslation()->z());
		sourceRotation[i].set(ext->getRotation()->x(), ext->getRotation()->y(), ext->getRotation()->z(), ext->getRotation()->w());
		auto cloudSource = sensors_[i]->getCloudSource();
		auto temp_cloud_ = cloudSource->getOutputCloud();
		if(temp_cloud_ && temp_cloud_->size() > 0){
			createIndexedOfMesh(temp_cloud_, i, inputMesh[i]);
		}
	}

	// set current cameras calibration in gui
	loadExtrinsicsFromCurrentSensor();
	updateGuiTransformation();


	// setup camera
	std::cout << "Setup camera" << std::endl;
	
	cam_.setPosition(ofVec3f(0, -10, 0));
	cam_.setFov(57);
	cam_.lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
	cam_.setNearClip(.1);
	cam_.setFarClip(100000000);

	// setup oF to Unity transformation
	ofToUnityTransformation.rotate(270, 1, 0, 0);


	ControlsOfxGui::getInstance().loadSettings();

	setBackground(127);
	//this->splashScreen.end();
	fullyInitialized = true;
}

//--------------------------------------------------------------
void ofApp::update(){
	//pcl::ScopeTime t("Update");
	std::cout << "UPDATE" << std::endl;

	if (ofGetFrameNum() == 1)
	{
		nextframe = false;
		//this->setup2();
	}
	else if (fullyInitialized) 
	{
		// Update Framerate in Gui
		//ControlsOfxGui::getInstance().updateFramerate(ofGetFrameRate());
		//pipeline_->processData();
		getNewFrame();
		//pipeline_->processData(currentFrame_);

		auto image = currentFrame_->getInputImage(0);
		if (image)
		{
			toOfTexture(image, texture_);
		}

		//createOfMeshFromPoints(pipeline_->getOutputCloud(), outputMesh);
		//createOfMeshFromPointsAndTriangles(pipeline_->getOutputCloud(), pipeline_->getTriangles(), outputMesh);
		for(auto i = 0; i < NCLOUDS; i++) 
		{

			//createIndexedOfMesh(currentFrame_->getInputCloud(i), i, inputMesh[i]);
			//createOfMeshFromPointsAndTriangles(currentFrame_->getOutputCloud(i), currentFrame_->getOutputTriangles(i), outputMesh[i]);

		}
	}
}

//--------------------------------------------------------------
void ofApp::drawReconstruction()
{

	switch(rendermode){
	case RENDER_SOURCES:
		for(auto i = 0; i < NCLOUDS; i++) {
			ofPushMatrix();
			ofPushStyle();
			if(ControlsOfxGui::getInstance().transformSources) {

				ofTranslate(sourceTranslation[i].x * 1000, sourceTranslation[i].y * 1000, sourceTranslation[i].z * 1000);

				ofVec3f qaxis; float qangle;
				sourceRotation[i].getRotate(qangle, qaxis);
				ofRotate(qangle, qaxis.x , qaxis.y, qaxis.z);
			}
			ofMultMatrix(ofToUnityTransformation);
			inputMesh[i].enableColors();
			inputMesh[i].drawVertices();
			ofPopMatrix();
			ofPopStyle();
		}
		break;
	case RENDER_POINTS:
		ofPushMatrix();
		ofPushStyle();
		for(auto &m : outputMesh){
			m.disableTextures();
			m.enableColors();
			ofMultMatrix(ofToUnityTransformation);
			m.drawVertices();
		}
		ofPopMatrix();
		ofPopStyle();
		break;
	case RENDER_WIRE:
		ofPushMatrix();
		ofPushStyle();
		for(auto &m : outputMesh){
			m.enableColors();
			ofMultMatrix(ofToUnityTransformation);
			m.drawWireframe();
		}
		ofPopMatrix();
		ofPopStyle();
		break;
	case RENDER_MESH:
		ofPushMatrix();
		ofPushStyle();
		for(auto &m : outputMesh){
			m.enableColors();
			m.disableTextures();
			ofMultMatrix(ofToUnityTransformation);
			m.draw();
		}
		ofPopMatrix();
		ofPopStyle();
		break;
	case RENDER_TEXTURE_MESH:
		ofPushMatrix();
		ofPushStyle();
		for(auto &m : outputMesh){
			m.disableColors();
			m.enableTextures();
			texture_.setTextureWrap(GL_CLAMP,GL_CLAMP);
			texture_.bind();
			ofMultMatrix(ofToUnityTransformation);
			m.draw();
			texture_.unbind();
		}
		ofPopMatrix();
		ofPopStyle();
		break;
	default:
		ofPushMatrix();
		ofPushStyle();
		for(auto &m : outputMesh){
			m.drawVertices();
		}
		ofPopMatrix();
		ofPopStyle();
		break;
	}


}

//--------------------------------------------------------------
void ofApp::drawCalibration()
{
	for(auto i = 0; i < NCLOUDS; i++) {
		ofPushMatrix();
		ofPushStyle();
		ofVec3f qaxis; float qangle;
		sourceRotation[i].getRotate(qangle, qaxis);
		if(ControlsOfxGui::getInstance().transformSources) {
			ofTranslate(sourceTranslation[i].x * 1000, sourceTranslation[i].y * 1000, sourceTranslation[i].z * 1000);
			ofRotate(qangle, qaxis.x , qaxis.y, qaxis.z);
		}
		if (i == selectedCamera) {
			inputMesh[i].disableColors();
			ofColor(255, 255, 255);
		}
		else
		{
			inputMesh[i].enableColors();
		}

		ofMultMatrix(ofToUnityTransformation);
		inputMesh[i].drawVertices();		
		ofDrawAxis(100);
		ofPopMatrix();
		ofPopStyle();

		//ofPushMatrix();
		//outputMesh.drawVertices();
		//ofPopMatrix();

		// camera frustum on active cam
		if (i == selectedCamera) {
			ofPushMatrix();
			ofPushStyle();
			ofMatrix4x4 mat, persp;
			mat.translate(-sourceTranslation[i].x * 1000, -sourceTranslation[i].y * 1000, -sourceTranslation[i].z * 1000);
			mat.rotate(qangle, -qaxis.x, -qaxis.y, -qaxis.z);
			mat.postMult(ofToUnityTransformation);

			persp.makePerspectiveMatrix(50, 1.33, .1, 100000);
			mat.postMult(persp);
			ofMultMatrix(mat.getInverse());
			ofNoFill();
			ofDrawBox(0, 0, 0, 2.0f);
			ofPopMatrix();
			ofPopStyle();
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	//pcl::ScopeTime t("Draw");
	std::cout << "UPDATE" << std::endl;

	ofBackground(background);

	cam_.begin();
	ofPushMatrix();
	ofPushStyle();
	ofDrawAxis(1000);

	if (appmode == APPMODE_RECON)
	{
		drawReconstruction();
	}
	else if (appmode == APPMODE_CONFIG)
	{
		drawCalibration();
	}

	ofPopMatrix();
	cam_.end();

	ofPushMatrix();
	ofPopStyle();
	auto aspect =  texture_.getWidth() / texture_.getHeight();
	auto screenWidth = ofGetWidth()/8;
	auto screenHeight = screenWidth / aspect;
	//                                                             vvv to flip image
	//texture.draw(ofPoint(screenWidth,ofGetHeight() - screenHeight), -screenWidth, screenHeight);
	ofPopMatrix();

	ofDisableDepthTest();
	ControlsOfxGui::getInstance().draw();
	ofEnableDepthTest();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if(key == ' ')
	{
		nextframe = true;
	}
	if(key == OF_KEY_F1)
	{
		ControlsOfxGui::getInstance().setStepHigh(true);
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
	if(key == OF_KEY_F1)
	{
		ControlsOfxGui::getInstance().setStepHigh(false);
	}
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

	// disable cam when mouse on mainGui
	if(ControlsOfxGui::getInstance().getGui()->getShape().getMaxX() >= x 
		&& ControlsOfxGui::getInstance().getGui()->getShape().getMaxY() >= y){
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
	ControlsOfxGui::getInstance().saveSettings();
	//for (auto &cs : cloudSource_) {
	//	cs->stop();
	//}
	SensorCalibrationSettings set;
	if (sensors_) {
		for (auto i = 0; i < NCLOUDS; i++)
		{
			set.saveCalibration(sensors_[i], i);
			i++;
		}
	}
}

//--------------------------------------------------------------
void ofApp::setBackground(float color){
	background = color;
}

//--------------------------------------------------------------
void ofApp::setAppmode(int mode)
{
	appmode = mode;
	if(mode == APPMODE_CONFIG) 
	{
		loadExtrinsicsFromCurrentSensor();
	}
	else if (mode == APPMODE_RECON)
	{
		saveExtrinsicsToCurrentSensor();
	}
}

//--------------------------------------------------------------
ofVec2f* ofApp::calculateTextureCoordinate(ofVec3f &point, int cam_index)
{
	ofVec3f qaxis; float qangle;
	ofMatrix4x4 mat, persp;
	auto width = texture_.getWidth();
	auto height = texture_.getHeight();
	auto intrinsics = sensors_[cam_index]->getDepthIntrinsics();
	//loadExtrinsicsFromCurrentSensor();
	sourceRotation[cam_index].getRotate(qangle, qaxis);

	mat.translate(sourceTranslation[cam_index].x*1000, sourceTranslation[cam_index].y*1000, sourceTranslation[cam_index].z*1000);
	mat.rotate(qangle, -qaxis.x, -qaxis.y, -qaxis.z);
	mat.postMult(ofToUnityTransformation);
	mat = mat.getInverse();
	persp.makePerspectiveMatrix(intrinsics->getVFov(), intrinsics->getAspectRatio(), .1, 100000);

	ofVec3f cameraSpacePoint;
	cameraSpacePoint = point;// * mat.getInverse();
	ofVec4f projectedPoint = persp.postMult(ofVec4f(cameraSpacePoint.x, cameraSpacePoint.y, cameraSpacePoint.z,1));// * persp;
	projectedPoint.x /= projectedPoint.w;
	projectedPoint.y /= projectedPoint.w;
	auto new_x = (projectedPoint.x * width) / (2 * projectedPoint.w) + (width * 0.5);
	auto new_y = (projectedPoint.y * height) / (2 * projectedPoint.w) + (height * 0.5);
	return new ofVec2f(new_x, new_y);
}

//--------------------------------------------------------------
void ofApp::setRendermode(int mode){
	rendermode = mode;
}

//--------------------------------------------------------------
void ofApp::createOfMeshFromPointsAndTriangles(recon::CloudConstPtr inputCloud, recon::TrianglesPtr triangles, ofMesh &targetMesh)
{
	if(triangles && inputCloud) {
		// triangle inputMesh
		targetMesh.clear();
		targetMesh.setMode(OF_PRIMITIVE_TRIANGLES);
		recon::PointType p;
		for(auto &t : *triangles) {
			// So easy, such style, very beauty, many readable, so wow!
			for(auto &pointindex : t.vertices){
				p = inputCloud->at(pointindex);
				ofVec3f ofp = ofVec3f(p.x*1000,p.y*1000,p.z*1000);
				targetMesh.addVertex(ofp);
				targetMesh.addColor(ofColor(p.r, p.g, p.b));
				//TODO: add normals, texturecoordinates
				ofVec3f source_point = inputMesh[0].getVertex(pointindex);
				targetMesh.addTexCoord(*calculateTextureCoordinate(source_point, 0));
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::createOfMeshFromPoints(recon::CloudConstPtr inputCloud, ofMesh &targetMesh)
{
	// triangle inputMesh
	targetMesh.clear();
	targetMesh.setMode(OF_PRIMITIVE_POINTS);
	for(auto &p : inputCloud->points){
		targetMesh.addVertex(ofVec3f(p.x*1000,p.y*1000,p.z*1000));
		targetMesh.addColor(ofColor(p.r,p.g,p.b));
		//targetMesh.addColor(cloudColors[meshIndex]);
	}
	//std::cout << "Mesh Size after meshing: " << targetMesh.getNumVertices() << " " << inputCloud->size() << std::endl;
}

//--------------------------------------------------------------
void ofApp::createIndexedOfMesh(recon::CloudConstPtr inputCloud, int meshIndex, ofMesh &targetMesh)
{
	if(inputCloud) {
		// triangle inputMesh
		targetMesh.clear();
		targetMesh.setMode(OF_PRIMITIVE_POINTS);
		for(auto &p : inputCloud->points){
			targetMesh.addVertex(ofVec3f(p.x*1000,p.y*1000,p.z*1000));
			targetMesh.addColor(cloudColors[meshIndex]);
		}
	}
}

//--------------------------------------------------------------
void ofApp::updateCameraTransformation(float xPos, float yPos, float zPos, float xRot, float yRot, float zRot)
{
	sourceTranslation[selectedCamera].set(xPos, yPos, zPos);

	ofVec3f x(1,0,0), y(0,1,0), z(0,0,1);
	ofQuaternion q;
	q.makeRotate(xRot, x, yRot ,y ,zRot ,z);
	sourceRotation[selectedCamera] = q;
}

//--------------------------------------------------------------
void ofApp::saveExtrinsicsToCurrentSensor()
{
	Eigen::Vector4f translation(sourceTranslation[selectedCamera].x, sourceTranslation[selectedCamera].y, sourceTranslation[selectedCamera].z, 0);
	Eigen::Quaternionf rotation(sourceRotation[selectedCamera].w(), sourceRotation[selectedCamera].x(), sourceRotation[selectedCamera].y(), sourceRotation[selectedCamera].z());
	recon::CameraExtrinsics::Ptr ext(new recon::CameraExtrinsics(translation, rotation));
	sensors_[selectedCamera]->setDepthExtrinsics(ext);
}

//--------------------------------------------------------------
void ofApp::updateGuiTransformation()
{
	ControlsOfxGui::getInstance().setCameraTransformation(sourceTranslation[selectedCamera].x,
	                                                sourceTranslation[selectedCamera].y,
	                                                sourceTranslation[selectedCamera].z,
	                                                sourceRotation[selectedCamera].getEuler().x,
	                                                sourceRotation[selectedCamera].getEuler().y,
	                                                sourceRotation[selectedCamera].getEuler().z);
}

//--------------------------------------------------------------
void ofApp::loadExtrinsicsFromCurrentSensor()
{
	if (sensors_) {
		auto ext = sensors_[selectedCamera]->getDepthExtrinsics();
		sourceTranslation[selectedCamera].set(ext->getTranslation()->x(), ext->getTranslation()->y(), ext->getTranslation()->z());
		sourceRotation[selectedCamera].set(ext->getRotation()->x(), ext->getRotation()->y(), ext->getRotation()->z(), ext->getRotation()->w());

		updateGuiTransformation();
	}
}

//--------------------------------------------------------------
void ofApp::selectNextCamera()
{
	saveExtrinsicsToCurrentSensor();

	selectedCamera = (selectedCamera + 1) % NCLOUDS;

	loadExtrinsicsFromCurrentSensor();
}

//--------------------------------------------------------------
void ofApp::setTexturesEnabled(bool state)
{
	textures_enabled = state;
}

//--------------------------------------------------------------
void ofApp::getNewFrame()
{
	currentFrame_ = boost::make_shared<recon::Frame>(NCLOUDS);
	for (auto &s : sensors_)
	{
		currentFrame_->setInputCloud(s->getCloudSource()->getOutputCloud(), s->getId());
		currentFrame_->setInputImage(s->getCloudSource()->getOutputImage(), s->getId());
		currentFrame_->setInputExtrinsics(s->getDepthExtrinsics(), s->getId());
		currentFrame_->setInputIntrinsics(s->getDepthIntrinsics(), s->getId());
	}
}

//--------------------------------------------------------------
void ofApp::updateFovOfCurrentCamera(float f)
{
	if (sensors_) {
		sensors_[selectedCamera]->getDepthIntrinsics()->setFocalLength(f);
	}
}