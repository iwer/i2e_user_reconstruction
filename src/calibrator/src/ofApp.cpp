#include "ofApp.h"
#include <of-pcl-bridge/of-pcl-bridge.h>

//--------------------------------------------------------------
void ofApp::setup(){
	recon::SensorFactory sensorFac;
	selected_sensor_ = -1;
	ofColor cloudColors[4];
	cloudColors[0].set(255, 0, 0);
	cloudColors[1].set(0, 255, 0);
	cloudColors[2].set(0, 0, 255);
	cloudColors[3].set(255, 255, 0);

	auto nSensors = sensorFac.checkConnectedDevices(true);
	for (int i = 0; i < nSensors; i++) {
		sensor_list_.push_back(sensorFac.createPclOpenNI2Grabber());
		cloudColor_[sensor_list_.back()->getId()] = cloudColors[i];
		loadExtrinsicsFromCurrentSensor();
	}
	sensor_list_it_ = sensor_list_.begin();

	ui_.setup();
	xTrans_.addListener(this, &ofApp::guiUpdatedExtrinsics);
	yTrans_.addListener(this, &ofApp::guiUpdatedExtrinsics);
	zTrans_.addListener(this, &ofApp::guiUpdatedExtrinsics);
	xRot_.addListener(this, &ofApp::guiUpdatedExtrinsics);
	yRot_.addListener(this, &ofApp::guiUpdatedExtrinsics); 
	zRot_.addListener(this, &ofApp::guiUpdatedExtrinsics);
	
	ui_.add(nextCamBtm_.setup("Next Camera"));
	ui_.add(prevCamBtm_.setup("Previous Camera"));
	ui_.add(xTransSl_.setup(xTrans_));
	ui_.add(yTransSl_.setup(yTrans_));
	ui_.add(zTransSl_.setup(zTrans_));
	ui_.add(xRotSl_.setup(xRot_));
	ui_.add(yRotSl_.setup(yRot_));
	ui_.add(zRotSl_.setup(zRot_));
}

//--------------------------------------------------------------
void ofApp::update(){
	for (auto &sensor : sensor_list_) {
		auto cloud = sensor->getCloudSource()->getOutputCloud();
		if (cloud != nullptr) {
			ofMesh mesh;
			createOfMeshFromPoints(cloud, cloudColor_[sensor->getId()], mesh);
			mesh_map_[sensor->getId()] = mesh;
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0);

	ofDrawBitmapString("fps: " + std::to_string(ofGetFrameRate()), 10, 10);

	cam_.begin();
	ofEnableDepthTest();
	ofDrawAxis(1000);

	
	for (auto &sensor : sensor_list_) {
		if(sensor->getId() == selected_sensor_)
		{
			mesh_map_[sensor->getId()].disableColors();
		}
		else
		{
			mesh_map_[sensor->getId()].enableColors();
		}
		ofPushMatrix();
		ofMultMatrix(sensor_extrinsics_[sensor->getId()]);

		mesh_map_[sensor->getId()].drawVertices();
		ofPopMatrix();
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

void ofApp::selectNextCamera()
{
	saveExtrinsicsToCurrentSensor();
	++sensor_list_it_;
	if(sensor_list_it_ == sensor_list_.end())
	{
		sensor_list_it_ = sensor_list_.begin();
	}
	selected_sensor_ = sensor_list_it_->get()->getId();
	loadExtrinsicsFromCurrentSensor();
}

void ofApp::selectPreviousCamera()
{
	saveExtrinsicsToCurrentSensor();
	if(sensor_list_it_ == sensor_list_.begin())
	{
		sensor_list_it_ = sensor_list_.end();
	}
	--sensor_list_it_;
	selected_sensor_ = sensor_list_it_->get()->getId();
	loadExtrinsicsFromCurrentSensor();
}

void ofApp::saveExtrinsicsToCurrentSensor()
{
	auto m = sensor_extrinsics_[selected_sensor_];
	auto t = m.getTranslation();
	auto q = m.getRotate();
	Eigen::Vector4f tE;
	Eigen::Quaternionf qE;
	toEigenVector4f(t, tE);
	toEigenQuaternionf(q, qE);
	recon::CameraExtrinsics::Ptr ext(new recon::CameraExtrinsics(tE, qE));
	(*sensor_list_it_)->setDepthExtrinsics(ext);
}

void ofApp::loadExtrinsicsFromCurrentSensor()
{
	auto ext = (*sensor_list_it_)->getDepthExtrinsics();
	ofVec3f t;
	ofQuaternion q;
	toOfVector3(*ext->getTranslation(), t);
	toOfQuaternion(*ext->getRotation(), q);

	ofMatrix4x4 m;
	m.translate(t);
	m.rotate(q);

	sensor_extrinsics_[selected_sensor_] = m;
	xTrans_ = t.x;
	yTrans_ = t.y;
	zTrans_ = t.z;
	xRot_ = q.getEuler().x;
	yRot_ = q.getEuler().y;
	zRot_ = q.getEuler().z;
}

void ofApp::guiUpdatedExtrinsics()
{
	ofVec3f x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
	ofVec3f v(xTrans_, yTrans_, zTrans_);
	ofQuaternion q;
	ofMatrix4x4 m;

	q.makeRotate(xRot_, x, yRot_, y, zRot_, z);

	m.translate(v);
	m.rotate(q);

	sensor_extrinsics_[selected_sensor_] = m;
}
