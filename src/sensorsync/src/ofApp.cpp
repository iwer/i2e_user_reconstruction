#include "ofApp.h"
#include <of-pcl-bridge/of-pcl-bridge.h>

//--------------------------------------------------------------
void ofApp::setup(){
	sensor1_ = new pcl::io::OpenNI2Grabber("#0");
	sensor2_ = new pcl::io::OpenNI2Grabber("#1");

	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) > cb1 = boost::bind(&ofApp::cloud_cb1, this, _1);
	boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) > cb2 = boost::bind(&ofApp::cloud_cb2, this, _1);

	sensor1_->registerCallback(cb1);
	sensor2_->registerCallback(cb2);
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
	cam_.begin();
	mesh1.drawVertices();
	mesh2.drawVertices();
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

void ofApp::cloud_cb1(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
{
	recon::Cloud alphaRemovedCloud;
	pcl::copyPointCloud<pcl::PointXYZRGBA, recon::PointType>(*cloud, alphaRemovedCloud);
	createOfMeshFromPoints(alphaRemovedCloud.makeShared(), mesh1);
}

void ofApp::cloud_cb2(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
{
	recon::Cloud alphaRemovedCloud;
	pcl::copyPointCloud<pcl::PointXYZRGBA, recon::PointType>(*cloud, alphaRemovedCloud);
	createOfMeshFromPoints(alphaRemovedCloud.makeShared(), mesh2);
}
