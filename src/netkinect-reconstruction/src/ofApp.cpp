#include "ofApp.h"
#include "NetKinectSensor.h"
#include <of-pcl-bridge/of-pcl-bridge.h>
#include <common/SensorCalibrationSettings.h>
#include <recon/SensorFactory.h>
#include <common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/time.h>
#include <pcl/io/image_rgb24.h>
#include <recon/CameraIntrinsics.h>

void ofApp::setupUi()
{
	backgroundRemovalPrms_.setName("Backgroundremoval");
	backgroundRemovalPrms_.add(passMin_);
	passMin_.addListener(this, &ofApp::processFrameTriggerFloat);
	backgroundRemovalPrms_.add(passMax_);
	passMax_.addListener(this, &ofApp::processFrameTriggerFloat);

	triangulationPrms_.setName("Triangulation");
	triangulationPrms_.add(triEdgeLength_);
	triEdgeLength_.addListener(this, &ofApp::processFrameTriggerInt);
	triangulationPrms_.add(angleTolerance_);
	angleTolerance_.addListener(this, &ofApp::processFrameTriggerFloat);
	triangulationPrms_.add(distanceTolerance_);
	distanceTolerance_.addListener(this, &ofApp::processFrameTriggerFloat);

	loadCalibrationBtn_.addListener(this, &ofApp::loadCalibrationFromFile);
	fps_.addListener(this, &ofApp::updateFps);
	backBtn_.addListener(this, &ofApp::back);
	playing_.addListener(this, &ofApp::play);
	stopBtn_.addListener(this, &ofApp::stop);
	nextFrameBtn_.addListener(this, &ofApp::nextFrame);
	prevFrameBtn_.addListener(this, &ofApp::prevFrame);
	saveCurrentFrame_.addListener(this, &ofApp::saveCurrentFrame);
	resetCalibrationBtn_.addListener(this, &ofApp::resetCalibration);

	ui_.setup();
	ui_.add(backgroundSl_.setup(background_));
	ui_.add(loadCalibrationBtn_.setup("Load Calibration"));
	ui_.add(resetCalibrationBtn_.setup("Reset Calibration"));
	ui_.add(saveCurrentFrame_.setup("Save current Mesh"));
	ui_.add(onlyPointsTgl_.setup("Only Points", true));
	ui_.add(fillWireFrameTgl_.setup("Fill Wireframe", true));
	ui_.add(perPixelColor_.setup("Per-Pixel Color", false));
	ui_.add(showFrustum_.setup("Show Frustum", false));
	ui_.add(showSingle_.setup("Show Single Meshes", true));
	ui_.add(showCombined_.setup("Show Combined Mesh", false));
	ui_.add(reconstructAllTgl_.setup("Reconstruct all", false));
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

	// KINECT SERVER ########
	std::cout << "Waiting for networked Kinect servers to connect" << std::endl;
	while(!netkinect_api_.isAbleToDeliverData()) {
		//std::cout << ".";
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	std::cout << std::endl << "Done. " << netkinect_api_.getClientCount() << " networked Kinects connected." << std::endl;
	auto sensorCount = netkinect_api_.getClientCount();

	recon::SensorFactory sensorFac;
	maxFrames_ = 1000000000;
	for (auto i = 0; i < sensorCount; i++)
	{
		auto s = boost::shared_ptr<recon::AbstractSensor>(new NetKinectSensor(netkinect_api_.getClient(i), i));

		double depth_focal_length_x, depth_focal_length_y, depth_principal_point_x, depth_principal_point_y;
		// this does not return anything!!!
		//grabber_->getDepthCameraIntrinsics(depth_focal_length_x, depth_focal_length_y, depth_principal_point_x, depth_principal_point_y);

		auto x_res = 640;
		auto y_res = 480;
		// default values
		depth_principal_point_x =  x_res / 2 - 0.5;
		depth_principal_point_y =  y_res / 2 - 0.5;

		depth_focal_length_x = 517.4;
		depth_focal_length_y = 517.4;

		recon::CameraIntrinsics intrinsics(depth_focal_length_x, depth_focal_length_y, depth_principal_point_x, depth_principal_point_y, x_res, y_res);

		s->setDepthIntrinsics(boost::make_shared<recon::CameraIntrinsics>(intrinsics));
		sensors_.push_back(s);
		sensorMap_[i] = s;
        image_[i] = std::make_shared<ofImage>();
				recon::CloudPtr newcloud(new recon::Cloud());
				cloud_[i] = newcloud;
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

	globalFrameNumber_ = 0;

	combinedTexture_.allocate(iWidth2, iHeight2);

	cam_.setFarClip(100000);
	cam_.rotate(180, 0, 1, 0);


	writeIndex_ = 0;
}

//--------------------------------------------------------------
void ofApp::processFrame()
{
	//pcl::ScopeTime t("Frame Processing");

	combinedMesh_.clear();
	combinedMesh_.setMode(OF_PRIMITIVE_TRIANGLES);

	for (auto &s : sensors_) {
		//if (cloud_[s->getId()]) {
			if (cloud_[s->getId()] != nullptr && image_[s->getId()] != nullptr)
			{
				// remove back- and foreground
				recon::CloudPtr cloud_wo_back(new recon::Cloud());
				removeBackground(cloud_[s->getId()], cloud_wo_back, passMin_, passMax_, true);

				//std::cout << cloud_[s->getId()]->at(0) << " " << cloud_[s->getId()]->at(1) << std::endl;

				// fast triangulation
				cloud_wo_back->width=640;
				cloud_wo_back->height=480;
				recon::TrianglesPtr tris(new std::vector<pcl::Vertices>());
				organizedFastMesh(cloud_wo_back, tris, triEdgeLength_, angleTolerance_, distanceTolerance_);
				//std::cout << tris->size() << std::endl;

				ofMesh m;
				createOfMeshWithTexCoords(cloud_wo_back, tris, image_[s->getId()]->getTexture(), sensorMap_[s->getId()], m);
				//createOfMeshFromPoints(cloud_wo_back, m);
				mesh_[s->getId()] = m;

				recon::CloudPtr cloud_transformed(new recon::Cloud());
				pcl::transformPointCloud(*cloud_wo_back, *cloud_transformed, s->getDepthExtrinsics()->getTransformation());
				ofMesh points;
				createOfMeshWithCombinedTexCoords(cloud_transformed, tris, image_[s->getId()]->getTexture(), imageLayout_[s->getId()], sensorMap_[s->getId()], points);
				combinedMesh_.append(points);
			} else {
				std::cerr << "Cloud or image null" << std::endl;
			}

		//} else {
		//	std::cerr << "No cloud" << std::endl;
		//}
	}
	//if (!reconstructAllTgl_) {
	//	t.reset();
	//}
}

//--------------------------------------------------------------
void ofApp::update(){
		//pcl::ScopeTime t("update()");
    // get cloud and image data from ServerAPI
    if(netkinect_api_.isAbleToDeliverData()) {
        for (int i = 0; i < netkinect_api_.getClientCount(); i++) {
  					//recon::CloudPtr newcloud(new recon::Cloud());

						cloudsize[i] = netkinect_api_.getClient(i)->getCloud(&clouddata[i], cloudsize[i], cloud_[i]);

            //Encode cloud into pcl::PointCloud<pcl::PointXYZ>
	          /*  for (int j = 0; j <= cloudsize[i] - 3; j += 3) {
	                recon::PointType p;
	                p.z = clouddata[i][j];
	                p.x = clouddata[i][j + 1];
	                p.y = -clouddata[i][j + 2];

									//if(p.x != 0 || p.y != 0 || p.z != 0) {
									//	std::cout << p.x << " " << p.y << " " << p.z <<std::endl;
									//}

	                newcloud->push_back(p);
	            } */

		            // save for collection
		            //cloud_[i] = newcloud;

		            imagesize[i] = netkinect_api_.getClient(i)->getVideo(&imagedata[i], imagesize[i]);

		            void* dataptr = (void*) imagedata[i];
		            image_[i] = std::shared_ptr<ofImage>(new ofImage());
		            image_[i]->setFromPixels(static_cast<const unsigned char *>(dataptr), 640, 480, OF_IMAGE_COLOR);

								netkinect_api_.getClient(i)->processedData();

        }
    }


//	if (reconstructAllTgl_)
//	{
//		saveCurrentFrame();
//		++globalFrameNumber_;
//		if(globalFrameNumber_ >= maxFrames_ - 1)
//		{
//			globalFrameNumber_ = 0;
//			reconstructAllTgl_ = false;
//		}
//	}
	processFrame();
}

//--------------------------------------------------------------
void ofApp::draw(){
	cam_.enableMouseInput();
	cam_.disableMouseMiddleButton();

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
		//fpsString.append(std::string("[") +
			//std::to_string(frameNumber_[s->getId()]) +
			//std::string("/") +
			//std::to_string(player_[s->getId()]->getNumberFrames()) +
			//std::string("]  : "));
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

			if (onlyPointsTgl_)
			{
				mesh_[s->getId()].enableColors();
				mesh_[s->getId()].disableTextures();
				mesh_[s->getId()].drawVertices();
			}
			else
			{
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
	//ofDrawBitmapString(fpsString, ofGetWidth() - 200, 10);
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
			//player_[s->getId()]->start();
		}
	}
	else
	{
		std::cout << "Play " << play << std::endl;
		for (auto &s : sensors_)
		{
			//player_[s->getId()]->stop();
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
//	for(auto &p : player_)
//	{
//		p.second->setFrameNumber(0);
//	}
}

//--------------------------------------------------------------
void ofApp::nextFrame()
{
	if(!playing_ && globalFrameNumber_ < (maxFrames_ - 1))
	{
		globalFrameNumber_++;
		processFrame();
	}
}

//--------------------------------------------------------------
void ofApp::prevFrame()
{
	if (!playing_ && globalFrameNumber_ > 0)
	{
		globalFrameNumber_--;
		processFrame();
	}
}

//--------------------------------------------------------------
void ofApp::saveCurrentFrame()
{
	ofPixels pixels;
	combinedTexture_.readToPixels(pixels);

	auto mesh_name = std::string("reconstructed/frame_") + fileNumber(writeIndex_) + std::string(".ply");
	auto image_name = std::string("reconstructed/frame_") + fileNumber(writeIndex_) + std::string(".png");

	combinedMesh_.save(mesh_name);
	ofSaveImage(pixels, image_name);
	++writeIndex_;
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

void ofApp::resetCalibration()
{
	for (auto& s : sensors_)
	{
		recon::CameraExtrinsics::Ptr ext(new recon::CameraExtrinsics());
		s->setDepthExtrinsics(ext);
	}
}

//--------------------------------------------------------------
void ofApp::cloudCallback(int frameNumber, int sensorIndex, recon::CloudPtr cloud, std::shared_ptr<ofImage> image)
{
	//std::lock_guard<std::mutex> lock(mapLock_);
//	frameNumber_[sensorIndex] = frameNumber;
	cloud_[sensorIndex].swap(cloud);
	image_[sensorIndex].swap(image);
}

//--------------------------------------------------------------
void ofApp::updateFps(int &fps)
{
	for (auto &s : sensors_) {
//		player_[s->getId()]->setFramesPerSecond(fps);
	}
}

//--------------------------------------------------------------
std::string ofApp::fileNumber(int number)
{
	std::ostringstream ss;
	ss << std::setw(5) << std::setfill('0') << number;
	return ss.str();
}

//--------------------------------------------------------------
void ofApp::processFrameTriggerInt(int & value)
{
	processFrame();
}

//--------------------------------------------------------------
void ofApp::processFrameTriggerFloat(float & value)
{
	processFrame();
}
