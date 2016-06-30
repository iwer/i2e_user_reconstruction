#include "ofApp.h"
#include <recon/SensorFactory.h>
#include <of-pcl-bridge/of-pcl-bridge.h>
#include <common/common.h>
#include <common/SensorCalibrationSettings.h>

//--------------------------------------------------------------
void ofApp::setup(){
	recon::SensorFactory sensorFac;
	auto nSensors = sensorFac.checkConnectedDevices(true);
	ofTexture img;
	ofMesh mesh;
	for (int i = 0; i < nSensors; i++) {
		sensor_list_.push_back(sensorFac.createPclOpenNI2Grabber());
		sensor_images_.insert(std::pair<int, ofTexture>(sensor_list_.back()->getId(), img));
		sensor_meshes_.insert(std::pair<int, ofMesh>(sensor_list_.back()->getId(), mesh));
		selected_sensor_ = sensor_list_.back()->getId();
	}

	cam_.setFarClip(10000);

	// gui
	loadCalibrationBtn_.addListener(this, &ofApp::loadCalibrationFromFile);
	
	ui_.setup();
	ui_.add(backgroundSl_.setup(background_));
	ui_.add(passMinSl_.setup(passMin_));
	ui_.add(passMaxSl_.setup(passMax_));
	ui_.add(resolutionSl_.setup(resolution_));
	ui_.add(triEdgeLengthSl_.setup(triEdgeLength_));
	ui_.add(angleToleranceSl_.setup(angleTolerance_));
	ui_.add(distanceToleranceSl_.setup(distanceTolerance_));
	ui_.add(loadCalibrationBtn_.setup("Load calibration"));
	ui_.add(fillWireFrameTgl_.setup("Fill Wireframe", true));

}

//--------------------------------------------------------------
void ofApp::update(){

	for (auto &sensor : sensor_list_) {
		auto image = sensor->getCloudSource()->getOutputImage();
		ofTexture tex;

		if (image != nullptr) {
			toOfTexture(image, tex);
			sensor_images_.erase(sensor->getId());
			sensor_images_.insert(std::pair<int, ofTexture>(sensor->getId(), tex));
		}
		else {
			std::cerr << "Got no image from sensor " << sensor->getId() << std::endl;
		}

		auto cloud = sensor->getCloudSource()->getOutputCloud();
		if(cloud != nullptr)
		{
			// remove back- and foreground
			recon::CloudPtr cloud_wo_back(new recon::Cloud());
			removeBackground(cloud, cloud_wo_back, passMin_, passMax_, true);

			// fast triangulation
			recon::TrianglesPtr tris(new std::vector<pcl::Vertices>());
			organizedFastMesh(cloud_wo_back, tris, triEdgeLength_, angleTolerance_, distanceTolerance_);


			ofMesh m;
			//createOfMeshFromPointsAndTriangles(cloud,tris, m);
			createOfMeshWithTexCoords(cloud_wo_back, tris, tex, sensor, m);
			//createOfMeshFromPoints(cloud, m);
			sensor_meshes_.erase(sensor->getId());
			sensor_meshes_.insert(std::pair<int, ofMesh>(sensor->getId(), m));
		}
		else {
			//std::cerr << "Got no cloud from sensor " << sensor->getId() << std::endl;
		}



	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(background_);
	ofDrawBitmapString("fps: " + std::to_string(ofGetFrameRate()), 10, 10);

	cam_.begin();
	ofEnableDepthTest();
	ofDrawAxis(1000);

	for (auto &sensor : sensor_list_) {
		ofPushMatrix();
		auto ext = sensor->getDepthExtrinsics();

		auto translation = toOfVector3(*ext->getTranslation());
		auto rotation = toOfQuaternion(*ext->getRotation());
		ofVec3f qaxis;
		float qangle;
		rotation.getRotate(qangle, qaxis);
		ofTranslate(translation);
		ofRotate(qangle, qaxis.x, qaxis.y, qaxis.z);
		sensor_images_[sensor->getId()].bind();
		if (fillWireFrameTgl_) {
			sensor_meshes_[sensor->getId()].draw();
		} else
		{
			sensor_meshes_[sensor->getId()].drawWireframe();
		}
		sensor_images_[sensor->getId()].unbind(); 

		ofPopMatrix();
		drawCameraFrustum(sensor);

	}
	cam_.end();

	ofDisableDepthTest();
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

void ofApp::loadCalibrationFromFile()
{
	SensorCalibrationSettings set;
	for (auto& s : sensor_list_)
	{
		set.loadCalibration(s, s->getId());
	}
}
