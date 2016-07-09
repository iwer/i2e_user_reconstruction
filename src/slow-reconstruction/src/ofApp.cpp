#include "ofApp.h"
#include <of-pcl-bridge/of-pcl-bridge.h>
#include <common/SensorCalibrationSettings.h>
#include <recon/SensorFactory.h>
#include <common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/common/time.h>

void ofApp::setupUi()
{
	backgroundRemovalPrms_.setName("Backgroundremoval");
	passMin_.addListener(this, &ofApp::processFrameTriggerFloat);
	backgroundRemovalPrms_.add(passMin_);
	passMax_.addListener(this, &ofApp::processFrameTriggerFloat);
	backgroundRemovalPrms_.add(passMax_);
	
	downsamplingPrms_.setName("Downsampling");
	resolution_.addListener(this, &ofApp::processFrameTriggerFloat);
	downsamplingPrms_.add(resolution_);

	normalCalcPrms_.setName("Normal Estimation");
	normalKNeighbours_.addListener(this, &ofApp::processFrameTriggerInt);
	normalCalcPrms_.add(normalKNeighbours_);

	smoothingPrms_.setName("Smoothing");
	smoothRadius_.addListener(this, &ofApp::processFrameTriggerFloat);
	smoothingPrms_.add(smoothRadius_);

	triangulationPrms_.setName("Triangulation");
	triEdgeLength_.addListener(this, &ofApp::processFrameTriggerFloat);
	triangulationPrms_.add(triEdgeLength_);
	searchRadius_.addListener(this, &ofApp::processFrameTriggerFloat);
	triangulationPrms_.add(searchRadius_);
	mu_.addListener(this, &ofApp::processFrameTriggerFloat);
	triangulationPrms_.add(mu_);
	maxNeighbours_.addListener(this, &ofApp::processFrameTriggerInt);
	triangulationPrms_.add(maxNeighbours_);
	maxSurfaceAngle_.addListener(this, &ofApp::processFrameTriggerFloat);
	triangulationPrms_.add(maxSurfaceAngle_);
	minAngle_.addListener(this, &ofApp::processFrameTriggerFloat);
	triangulationPrms_.add(minAngle_);
	maxAngle_.addListener(this, &ofApp::processFrameTriggerFloat);
	triangulationPrms_.add(maxAngle_);

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
	ui_.add(showNormals_.setup("Show normals", false));
	ui_.add(fpsSlider_.setup(fps_));
	//ui_.add(backBtn_.setup("Rewind"));
	//ui_.add(playTgl_.setup(playing_));
	//ui_.add(stopBtn_.setup("Stop"));
	ui_.add(nextFrameBtn_.setup("Next Frame"));
	ui_.add(prevFrameBtn_.setup("Previous Frame"));
	ui_.add(backgroundRemovalPrms_);
	ui_.add(normalCalcPrms_);
	ui_.add(downsamplingPrms_);
	ui_.add(smoothingPrms_);
	ui_.add(triangulationPrms_);
}

//--------------------------------------------------------------
void ofApp::setup() {
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


	for (auto &r : imageLayout_)
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
	processFrame();
}


void ofApp::processFrame()
{
	pcl::ScopeTime time("Frame processing time");
	combinedCloud_.reset(new recon::NormalCloud());

	for (auto &s : sensors_) {
		if (!playing_)
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
				removeBackground(cloud_[s->getId()], cloud_wo_back, passMin_, passMax_, false);

				// calculate normals
				pcl::PointCloud<recon::PointNormalType>::Ptr cloud_with_normals(new recon::NormalCloud);
				calculatePointNormals(cloud_wo_back, cloud_with_normals, normalKNeighbours_);

				// transform pointcloud
				cloud_transformed_[s->getId()].reset(new recon::NormalCloud());
				pcl::transformPointCloud(*cloud_with_normals, *cloud_transformed_[s->getId()], s->getDepthExtrinsics()->getTransformation());

				// merge pointclouds
				*combinedCloud_ += *cloud_transformed_[s->getId()];
			}
		}
	}
	
	if (combinedCloud_->size() > 0) {
		// Downsample combined cloud
		recon::NormalCloudPtr cloud_downsampled(new recon::NormalCloud());
		downsample(combinedCloud_, cloud_downsampled, resolution_);

		// smoothing
		recon::NormalCloudPtr cloud_smoothed(new recon::NormalCloud());
		movingLeastSquaresSmoothing(cloud_downsampled, cloud_smoothed, smoothRadius_);

		// generate texture coordinate wrt the closest visible camera, this does not work very well...
		//std::vector<ofVec2f> tex_coords;
		//generateTextureCoordinates(cloud_smoothed, tex_coords, sensorMap_);

		// triangulate using greedy projection
		recon::TrianglesPtr tris(new std::vector<pcl::Vertices>());
		greedyProjectionMesh(cloud_smoothed, tris, triEdgeLength_, mu_, maxNeighbours_, maxSurfaceAngle_, minAngle_, maxAngle_);



		// pcl texture mapping
		tmesh_.reset(new pcl::TextureMesh());
		pcl::toPCLPointCloud2(*cloud_smoothed, tmesh_->cloud);
		tmesh_->tex_polygons.push_back(*tris);


		pcl::texture_mapping::CameraVector cam_vec;
		for (auto &s : sensors_) {
			cam_vec.push_back(s->asPclCamera());
		}

		//pcl::TexMaterial material;
		//material.
		pcl::TextureMapping<pcl::PointXYZ> tmapping;
		tmapping.textureMeshwithMultipleCameras(*tmesh_, cam_vec);
	}
}

void ofApp::processFrameTriggerInt(int & value)
{
	processFrame();
}

void ofApp::processFrameTriggerFloat(float & value)
{
	processFrame();
}

//--------------------------------------------------------------
void ofApp::update() {
	combinedMesh_.clear();
	combinedMesh_.setMode(OF_PRIMITIVE_TRIANGLES);

	createOfMeshFromPclTextureMesh(tmesh_, imageLayout_, sensorMap_, combinedMesh_);
	//createOfMeshFromPointsWNormalsAndTriangles(cloud_smoothed, tris, combinedMesh_);
	//createOfMeshWithCombinedTexCoords(cloud_smoothed, tris, tex_coords, combinedMesh_);

	for(auto &s : sensors_)
	{
		// create ofMesh for displaying
		ofMesh m;
		createOfMeshFromPointsWNormals(cloud_transformed_[s->getId()], m);
		mesh_[s->getId()] = m;
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
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
			if(showNormals_)
			{
				drawNormals(mesh_[s->getId()], 10, false);
			}
			ofPopMatrix();
		}
		if (showFrustum_)
			drawCameraFrustum(s);
	}

	if (showCombined_) {
		if (fillWireFrameTgl_) {
			if (perPixelColor_)
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
		}else
		{
			combinedMesh_.drawWireframe();
		}
		if (showNormals_)
		{
			drawNormals(combinedMesh_, 10, false);
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
void ofApp::keyPressed(int key) {

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y) {

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {

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
	for (auto &p : player_)
	{
		p.second->setFrameNumber(0);
	}
}

//--------------------------------------------------------------
void ofApp::nextFrame()
{
	if (!playing_ && globalFrameNumber_ < (maxFrames_ - 1))
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
	processFrame();
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


void ofApp::drawNormals(ofMesh &mesh, float length, bool bFaceNormals) {

	if (mesh.usingNormals()) {
		const vector<ofVec3f>& normals = mesh.getNormals();
		const vector<ofVec3f>& vertices = mesh.getVertices();
		ofVec3f normal;
		ofVec3f vert;
		ofMesh normalsMesh;
		normalsMesh.setMode(OF_PRIMITIVE_LINES);
		normalsMesh.getVertices().resize(normals.size() * 2);

		if (bFaceNormals) {
			for (int i = 0; i < (int)normals.size(); i++) {
				if (i % 3 == 0) {
					vert = (vertices[i] + vertices[i + 1] + vertices[i + 2]) / 3;
				}
				else if (i % 3 == 1) {
					vert = (vertices[i - 1] + vertices[i] + vertices[i + 1]) / 3;
				}
				else if (i % 3 == 2) {
					vert = (vertices[i - 2] + vertices[i - 1] + vertices[i]) / 3;
				}
				normalsMesh.setVertex(i * 2, vert);
				normal = normals[i].getNormalized();
				normal *= length;
				normalsMesh.setVertex(i * 2 + 1, normal + vert);
			}
		}
		else {
			for (int i = 0; i < (int)normals.size(); i++) {
				vert = vertices[i];
				normal = normals[i].getNormalized();
				normalsMesh.setVertex(i * 2, vert);
				normal *= length;
				normalsMesh.setVertex(i * 2 + 1, normal + vert);
			}
		}
		normalsMesh.draw();
	}
}


//--------------------------------------------------------------
void ofApp::generateTextureCoordinates(recon::NormalCloudPtr pointCloud, vector<ofVec2f>& tex_coords, map<int, boost::shared_ptr<recon::AbstractSensor>>& sensors)
{
	for(auto &p : pointCloud->points)
	{
		auto normal = ofVec3f(p.normal_x, p.normal_y, p.normal_z);
		auto cam_id = selectClosestFacingCamera(normal, sensors);
		auto ofp = ofVec3f(p.x * 1000, p.y * 1000, p.z * 1000);

		switch(cam_id)
		{
		case 0:
			p.r = 255;
			p.g = 0;
			p.b = 0;
			break;
		case 1:
			p.r = 0;
			p.g = 255;
			p.b = 0;
			break;
		case 2:
			p.r = 0;
			p.g = 0;
			p.b = 255;
			break;
		default:
			break;
		}
		auto tex_coord = calculateTextureCoordinate(ofp, imageLayout_[cam_id].getWidth(), imageLayout_[cam_id].getHeight(), sensors[cam_id], false);
		tex_coord += imageLayout_[cam_id].getTopLeft();

		tex_coords.push_back(tex_coord);
	}
}

//--------------------------------------------------------------
int ofApp::selectClosestFacingCamera(ofVec3f& normal, map<int, boost::shared_ptr<recon::AbstractSensor>>& sensors)
{
	auto minAngle = 360.0f;
	int minAngleSensorId = 0;
	for(auto &s : sensors)
	{
		auto inverse_camera_transform = s.second->getDepthExtrinsics()->getTransformation().inverse();
		auto view_vector = ofVec3f(0, 0, 1) * toOfMatrix4x4(inverse_camera_transform);

		auto norm_n = normal.normalized();
		auto angle = norm_n.angle(view_vector);

		angle = std::abs(angle - 180);

		if(angle < minAngle)
		{
			minAngle = angle;
			minAngleSensorId = s.second->getId();
		}
	}
	return minAngleSensorId;
}

std::string ofApp::fileNumber(int number)
{
	std::ostringstream ss;
	ss << std::setw(5) << std::setfill('0') << number;
	return ss.str();
}

