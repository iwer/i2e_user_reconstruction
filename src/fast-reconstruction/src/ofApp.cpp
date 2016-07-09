#include "ofApp.h"
#include <of-pcl-bridge/of-pcl-bridge.h>
#include <common/SensorCalibrationSettings.h>
#include <recon/SensorFactory.h>
#include <common/common.h>
#include <pcl/common/transforms.h>

void ofApp::setupUi()
{
	backgroundRemovalPrms_.setName("Backgroundremoval");
	backgroundRemovalPrms_.add(passMin_);
	backgroundRemovalPrms_.add(passMax_);

	triangulationPrms_.setName("Triangulation");
	triangulationPrms_.add(triEdgeLength_);
	triangulationPrms_.add(angleTolerance_);
	triangulationPrms_.add(distanceTolerance_);

	loadCalibrationBtn_.addListener(this, &ofApp::loadCalibrationFromFile);
	fps_.addListener(this, &ofApp::updateFps);
	backBtn_.addListener(this, &ofApp::back);
	playing_.addListener(this, &ofApp::play); 
	stopBtn_.addListener(this, &ofApp::stop);
	nextFrameBtn_.addListener(this, &ofApp::nextFrame);
	prevFrameBtn_.addListener(this, &ofApp::prevFrame);
	saveCurrentFrame_.addListener(this, &ofApp::saveCurrentFrame);

	ui_.setup();
	ui_.add(backgroundSl_.setup(background_));
	ui_.add(loadCalibrationBtn_.setup("Load Calibration"));
	ui_.add(saveCurrentFrame_.setup("Save current Mesh"));
	ui_.add(fillWireFrameTgl_.setup("Fill Wireframe", true));
	ui_.add(perPixelColor_.setup("Per-Pixel Color", false));
	ui_.add(showFrustum_.setup("Show Frustum", false)); 
	ui_.add(showSingle_.setup("Show Single Meshes", true));
	ui_.add(showCombined_.setup("Show Combined Mesh", false));
	ui_.add(fpsSlider_.setup(fps_));
	ui_.add(backBtn_.setup("Rewind"));
	ui_.add(playTgl_.setup(playing_));
	ui_.add(stopBtn_.setup("Stop"));
	ui_.add(nextFrameBtn_.setup("Next Frame"));
	ui_.add(prevFrameBtn_.setup("Previous Frame"));
	ui_.add(backgroundRemovalPrms_);
	ui_.add(triangulationPrms_);
}

//--------------------------------------------------------------
void ofApp::setup(){
	setupUi();

	auto full_path(boost::filesystem::current_path());
	boost::filesystem::path recorder_data_path("./data/recorder");

	auto sensorCount = PointCloudPlayer::getNumberSensors(full_path.generic_string() + recorder_data_path.generic_string());
	std::cout << "Sensor count: " << sensorCount << std::endl;

	recon::SensorFactory sensorFac;
	maxFrames_ = 1000000000;
	for (auto i = 0; i < sensorCount; i++)
	{
		auto s = sensorFac.createDummySensor();
		sensors_.push_back(s);
		sensorMap_[s->getId()] = s;

		player_[s->getId()] = PointCloudPlayer::Ptr(new PointCloudPlayer(full_path.generic_string() + recorder_data_path.generic_string(),
			s->getId(), 1));
		player_[s->getId()]->callback.connect(boost::bind(&ofApp::cloudCallback, this, _1, _2, _3, _4));
		image_[s->getId()] = std::make_shared<ofImage>();

		maxFrames_ = std::min(maxFrames_, player_[s->getId()]->getNumberFrames());
	}


	int iWidth = 640;
	int iHeight = 480;
	int iWidth2 = iWidth * 2;
	int iHeight2 = iHeight * 2;
	switch (sensorCount) {
	case 1:
		imageLayout_.push_back(ofRectangle(0, 0, iWidth2, iHeight2));
	case 2:
	case 3:
	case 4:
		imageLayout_.push_back(ofRectangle(0, 0, iWidth, iHeight));
		imageLayout_.push_back(ofRectangle(iWidth, 0, iWidth, iHeight));
		imageLayout_.push_back(ofRectangle(0, iHeight, iWidth, iHeight));
		imageLayout_.push_back(ofRectangle(iWidth, iHeight, iWidth, iHeight));
	}


	for(auto &r: imageLayout_)
	{
		std::cout << r << std::endl;
	}
	// TODO: Fix Texture bug to get rid of dummy Texture
	dummyTex_.load("uv_test.png");
	globalFrameNumber_ = 0;
	
	combinedTexture_.allocate(iWidth2, iHeight2);

	cam_.setFarClip(100000);
	cam_.rotate(180, 0, 1, 0);
	cam_.enableMouseInput();
	cam_.disableMouseMiddleButton();

	//loadCalibrationFromFile();
}

//--------------------------------------------------------------
void ofApp::update(){
	combinedMesh_.clear();
	combinedMesh_.setMode(OF_PRIMITIVE_TRIANGLES);

	for (auto &s : sensors_) {
		if(!playing_)
		{
			// aquire frame with globalFrameNumber_
			auto framedata = player_[s->getId()]->requestFrame(globalFrameNumber_);
			cloudCallback(globalFrameNumber_, s->getId(), framedata->cloud_, framedata->image_);
		}
		if (cloud_[s->getId()]) {
			if (cloud_[s->getId()] != nullptr && image_[s->getId()] != nullptr)
			{
				// remove back- and foreground
				recon::CloudPtr cloud_wo_back(new recon::Cloud());
				removeBackground(cloud_[s->getId()], cloud_wo_back, passMin_, passMax_, true);

				// fast triangulation
				recon::TrianglesPtr tris(new std::vector<pcl::Vertices>());
				organizedFastMesh(cloud_wo_back, tris, triEdgeLength_, angleTolerance_, distanceTolerance_);


				ofMesh m;
				createOfMeshWithTexCoords(cloud_wo_back, tris, image_[s->getId()]->getTexture(), sensorMap_[s->getId()], m);
				mesh_[s->getId()] = m;

				recon::CloudPtr cloud_transformed(new recon::Cloud());
				pcl::transformPointCloud(*cloud_wo_back, *cloud_transformed, s->getDepthExtrinsics()->getTransformation());
				ofMesh points;
				createOfMeshWithCombinedTexCoords(cloud_transformed, tris, image_[s->getId()]->getTexture(), imageLayout_[s->getId()], sensorMap_[s->getId()], points);
				combinedMesh_.append(points);
				std::cout << "sensors Mesh: " << points.getNumVertices() << " " << points.getNumTexCoords() <<  std::endl;
			}
		}
	}


}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(background_);
	ofEnableDepthTest();

	std::string fpsString;

	// combine textures
	combinedTexture_.begin();
	ofClear(64, 64, 64, 128);
	for (auto &s : sensors_) {
		ofPushMatrix();
		//ofTranslate(combinedTexture_.getWidth(), 0);
		//ofScale(-1, 1, 1);
		if (image_[s->getId()]->isAllocated()) {
			image_[s->getId()]->getTexture().draw(imageLayout_[s->getId()]);
		}
		ofPopMatrix();
	}
	combinedTexture_.end();

	// render 3d objects
	cam_.begin();
	ofDrawAxis(10);

	// per sensor rendering
	for (auto &s : sensors_) {
		fpsString.append(std::string("[") +
			std::to_string(frameNumber_[s->getId()]) +
			std::string("/") +
			std::to_string(player_[s->getId()]->getNumberFrames()) +
			std::string("]  : "));
		if (showSingle_) {
			ofPushMatrix();
			auto ext = s->getDepthExtrinsics();
			auto translation = toOfVector3(*ext->getTranslation());
			auto rotation = toOfQuaternion(*ext->getRotation());
			ofVec3f qaxis;
			float qangle;
			rotation.getRotate(qangle, qaxis);
			ofTranslate(translation);
			ofRotate(qangle, qaxis.x, qaxis.y, qaxis.z);
			if (fillWireFrameTgl_) {
				if (perPixelColor_)
				{
					image_[s->getId()]->getTexture().bind();
					mesh_[s->getId()].disableColors();
					mesh_[s->getId()].enableTextures();
					mesh_[s->getId()].draw();
					image_[s->getId()]->getTexture().unbind();
				}
				else {
					mesh_[s->getId()].enableColors();
					mesh_[s->getId()].disableTextures();
					mesh_[s->getId()].draw();
				}
			}
			else
			{
				mesh_[s->getId()].enableColors();
				mesh_[s->getId()].disableTextures();
				mesh_[s->getId()].drawWireframe();
			}
			ofPopMatrix();
		}
		if (showFrustum_)
			drawCameraFrustum(s);
	}

	if (showCombined_) {
		if(perPixelColor_)
		{
			combinedMesh_.enableTextures();
			combinedMesh_.disableColors();
			combinedTexture_.getTexture().bind();
			combinedMesh_.draw();
			combinedTexture_.getTexture().unbind();
		}
		else {
			combinedMesh_.disableTextures();
			combinedMesh_.enableColors();
			combinedMesh_.draw();
		}
	
	}

	cam_.end();


	ofPushMatrix();
	ofTranslate(ofGetWidth(), 0);
	ofScale(-1, 1, 1);
	combinedTexture_.draw(ofGetWidth() / 4 * 3, ofGetHeight() / 4 * 3, ofGetWidth() / 4, ofGetHeight() / 4);
	ofPopMatrix();

	ofDisableDepthTest();
	ofDrawBitmapString(fpsString, ofGetWidth() - 200, 10);
	ui_.draw();
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

//--------------------------------------------------------------
void ofApp::play(bool &play)
{
	if (play)
	{
		std::cout << "Play " << play << std::endl;
		for (auto &s : sensors_)
		{
			player_[s->getId()]->start();
		}
	}
	else
	{
		std::cout << "Play " << play << std::endl;
		for (auto &s : sensors_)
		{
			player_[s->getId()]->stop();
		}
	}
}

//--------------------------------------------------------------
void ofApp::stop()
{
	std::cout << "Stop" << std::endl;
	playTgl_ = false;
}

//--------------------------------------------------------------
void ofApp::back()
{
	std::cout << "Back" << std::endl;
	for(auto &p : player_)
	{
		p.second->setFrameNumber(0);
	}
}

//--------------------------------------------------------------
void ofApp::nextFrame()
{
	if(!playing_ && globalFrameNumber_ < (maxFrames_ - 1))
	{
		globalFrameNumber_++;
	}
}

//--------------------------------------------------------------
void ofApp::prevFrame()
{
	if (!playing_ && globalFrameNumber_ > 0)
	{
		globalFrameNumber_--;
	}
}

//--------------------------------------------------------------
void ofApp::saveCurrentFrame()
{
	combinedMesh_.save("combined.ply");
	ofPixels pixels;
	combinedTexture_.readToPixels(pixels);
	ofSaveImage(pixels, "combined.png");
}

//--------------------------------------------------------------
void ofApp::loadCalibrationFromFile()
{
	SensorCalibrationSettings set;
	for (auto& s : sensors_)
	{
		set.loadCalibration(s, s->getId());
		auto ext = s->getDepthExtrinsics();
		ofVec3f t;
		ofQuaternion q;
		toOfVector3(*ext->getTranslation(), t);
		toOfQuaternion(*ext->getRotation(), q);

		ofMatrix4x4 m;
		m.translate(t);
		m.rotate(q);

		//sensor_extrinsics_[s->getId()] = m;
	}
}

//--------------------------------------------------------------
void ofApp::cloudCallback(int frameNumber, int sensorIndex, recon::CloudPtr cloud, std::shared_ptr<ofImage> image)
{
	//std::lock_guard<std::mutex> lock(mapLock_);
	frameNumber_[sensorIndex] = frameNumber;
	cloud_[sensorIndex].swap(cloud);
	image_[sensorIndex].swap(image);
}

//--------------------------------------------------------------
void ofApp::updateFps(int &fps)
{
	for (auto &s : sensors_) {
		player_[s->getId()]->setFramesPerSecond(fps);
	}
}

//--------------------------------------------------------------
