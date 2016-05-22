#include "ofApp.h"
#include <of-pcl-bridge/of-pcl-bridge.h>
#include <common/SensorCalibrationSettings.h>
#include <common/common.h>

ofApp::ofApp()
	: xTrans_("X Translation", 0, -5, 5)
	, yTrans_("Y Translation", 0, -5, 5)
	, zTrans_("Z Translation", 0, -5, 5)
	, xRot_("X Rotation", 0, -180, 180)
	, yRot_("Y Rotation", 0, -180, 180)
	, zRot_("Z Rotation", 0, -180, 180)
	, passMin_("Z Min", .01, .01, 8)
	, passMax_("Z Max", 2.5, .01, 8)
	, triEdgeLength_("Triangle Edge Length", 5, 1, 50)
	, angleTolerance_("Angle Tolerance", 15, 1, 180)
	, distanceTolerance_("Distance Tolerance", .2, .001, .5)
{}
//--------------------------------------------------------------
void ofApp::setup(){
	recon::SensorFactory sensorFac;
	selected_sensor_id_ = 0;
	ofColor cloudColors[4];
	cloudColors[0].set(255, 0, 0);
	cloudColors[1].set(0, 255, 0);
	cloudColors[2].set(0, 0, 255);
	cloudColors[3].set(255, 255, 0);


	sensorCount_ = sensorFac.checkConnectedDevices(true);
	for (int i = 0; i < sensorCount_; i++) {
		auto sensor = sensorFac.createPclOpenNI2Grabber();
		sensorIds_.push_back(sensor->getId());
		sensor_list_[sensor->getId()] = sensor;
		cloudColor_[sensor->getId()] = cloudColors[i];
		sensor_extrinsics_[sensor->getId()] = ofMatrix4x4();
		loadExtrinsicsFromCurrentSensor();
	}

	// setup ui
	ui_.setup();
	xTrans_.addListener(this, &ofApp::guiUpdatedExtrinsics);
	yTrans_.addListener(this, &ofApp::guiUpdatedExtrinsics);
	zTrans_.addListener(this, &ofApp::guiUpdatedExtrinsics);
	xRot_.addListener(this, &ofApp::guiUpdatedExtrinsics);
	yRot_.addListener(this, &ofApp::guiUpdatedExtrinsics);
	zRot_.addListener(this, &ofApp::guiUpdatedExtrinsics);
	saveCalibrationBtn_.addListener(this, &ofApp::saveCalibrationToFile);
	loadCalibrationBtn_.addListener(this, &ofApp::loadCalibrationFromFile);
	
	ui_.add(nextCamBtm_.setup("Next Camera"));
	ui_.add(prevCamBtm_.setup("Previous Camera"));
	ui_.add(xTransSl_.setup(xTrans_));
	ui_.add(yTransSl_.setup(yTrans_));
	ui_.add(zTransSl_.setup(zTrans_));
	ui_.add(xRotSl_.setup(xRot_));
	ui_.add(yRotSl_.setup(yRot_));
	ui_.add(zRotSl_.setup(zRot_));

	ui_.add(passMinSl_.setup(passMin_));
	ui_.add(passMaxSl_.setup(passMax_));
	ui_.add(triEdgeLengthSl_.setup(triEdgeLength_));
	ui_.add(angleToleranceSl_.setup(angleTolerance_));
	ui_.add(distanceToleranceSl_.setup(distanceTolerance_));

	ui_.add(saveCalibrationBtn_.setup("Save calibration"));
	ui_.add(loadCalibrationBtn_.setup("Load calibration"));

	cam_.setFarClip(100000);
}

//--------------------------------------------------------------
void ofApp::update(){
	for (auto &sensor : sensor_list_) {
		auto image = sensor.second->getCloudSource()->getOutputImage();
		ofTexture tex;

		if (image != nullptr) {
			toOfTexture(image, tex);
			image_map_.erase(sensor.second->getId());
			image_map_.insert(std::pair<int, ofTexture>(sensor.second->getId(), tex));
		}
		else {
			std::cerr << "Got no image from sensor " << sensor.second->getId() << std::endl;
		}

		auto cloud = sensor.second->getCloudSource()->getOutputCloud();
		if (cloud != nullptr) {
			// remove back- and foreground
			recon::CloudPtr cloud_wo_back(new recon::Cloud());
			removeBackground(cloud, cloud_wo_back, passMin_, passMax_);

			// fast triangulation
			recon::TrianglesPtr tris(new std::vector<pcl::Vertices>());
			organizedFastMesh(cloud_wo_back, tris, triEdgeLength_, angleTolerance_, distanceTolerance_);

			ofMesh mesh;
			//createOfMeshFromPoints(cloud, cloudColor_[sensor.second->getId()], mesh);
			createOfMeshWithTexCoords(cloud_wo_back, tris, tex, sensor.second, mesh);
			mesh_map_.erase(sensor.second->getId());
			mesh_map_[sensor.second->getId()] = mesh;
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
		if(sensor.second->getId() == sensorIds_[selected_sensor_id_])
		{
			mesh_map_[sensor.second->getId()].disableColors();
		}
		else
		{
			mesh_map_[sensor.second->getId()].enableColors();
		}
		ofPushMatrix();
		auto ext = sensor.second->getDepthExtrinsics();
		auto translation = toOfVector3(*ext->getTranslation());
		auto rotation = toOfQuaternion(*ext->getRotation());
		ofVec3f qaxis;
		float qangle;
		rotation.getRotate(qangle, qaxis);
		ofTranslate(translation);
		ofRotate(qangle, qaxis.x, qaxis.y, qaxis.z);

		image_map_[sensor.second->getId()].bind();
		mesh_map_[sensor.second->getId()].draw();
		image_map_[sensor.second->getId()].unbind();

		ofPopMatrix();
		drawCameraFrustum(sensor.second);
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
		selected_sensor_id_ = (selected_sensor_id_ + 1) % sensorCount_;
		loadExtrinsicsFromCurrentSensor();
}

void ofApp::selectPreviousCamera()
{
		saveExtrinsicsToCurrentSensor();
		selected_sensor_id_ = (sensorCount_ + selected_sensor_id_ - 1) % sensorCount_;
		loadExtrinsicsFromCurrentSensor();
}

void ofApp::saveExtrinsicsToCurrentSensor()
{
		auto m = sensor_extrinsics_[sensorIds_[selected_sensor_id_]];
		auto t = m.getTranslation();
		auto q = m.getRotate();
		recon::CameraExtrinsics::Ptr ext(new recon::CameraExtrinsics(toEigenVector4f(t), toEigenQuaternionf(q)));
		sensor_list_[sensorIds_[selected_sensor_id_]]->setDepthExtrinsics(ext);
}

void ofApp::loadExtrinsicsFromCurrentSensor()
{
		auto ext = sensor_list_[sensorIds_[selected_sensor_id_]]->getDepthExtrinsics();
		ofVec3f t;
		ofQuaternion q;
		toOfVector3(*ext->getTranslation(), t);
		toOfQuaternion(*ext->getRotation(), q);

		ofMatrix4x4 m;
		m.translate(t);
		m.rotate(q);

		sensor_extrinsics_[sensorIds_[selected_sensor_id_]] = m;
		xTrans_ = t.x;
		yTrans_ = t.y;
		zTrans_ = t.z;
		xRot_ = q.getEuler().x;
		yRot_ = q.getEuler().y;
		zRot_ = q.getEuler().z;
}

void ofApp::guiUpdatedExtrinsics(float &dummy)
{
	ofVec3f x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
	// gui is in meters, oF scale is mm
	ofVec3f v(xTrans_ * 1000, yTrans_ * 1000, zTrans_ * 1000);
	ofQuaternion q;
	ofMatrix4x4 m;

	q.makeRotate(xRot_, x, yRot_, y, zRot_, z);

	m.translate(v);
	m.rotate(q);

	sensor_extrinsics_[sensorIds_[selected_sensor_id_]] = m;
	saveExtrinsicsToCurrentSensor();
}

void ofApp::loadCalibrationFromFile()
{
	SensorCalibrationSettings set;
	for (auto& s : sensor_list_)
	{
		set.loadCalibration(s.second, s.second->getId());
		auto ext = s.second->getDepthExtrinsics();
		ofVec3f t;
		ofQuaternion q;
		toOfVector3(*ext->getTranslation(), t);
		toOfQuaternion(*ext->getRotation(), q);

		ofMatrix4x4 m;
		m.translate(t);
		m.rotate(q);

		sensor_extrinsics_[sensorIds_[s.second->getId()]] = m;
	}
}

void ofApp::saveCalibrationToFile()
{
	SensorCalibrationSettings set;

	for (auto& s : sensor_list_)
	{
		auto m = sensor_extrinsics_[sensorIds_[s.second->getId()]];
		auto t = m.getTranslation();
		auto q = m.getRotate();
		recon::CameraExtrinsics::Ptr ext(new recon::CameraExtrinsics(toEigenVector4f(t), toEigenQuaternionf(q)));
		sensor_list_[sensorIds_[s.second->getId()]]->setDepthExtrinsics(ext);
		set.saveCalibration(s.second, s.second->getId());
	}
}