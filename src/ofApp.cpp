#include "ofApp.h"

ofApp::ofApp() :
	vertices_()
{
	grabber_ =  new pcl::io::OpenNI2Grabber("", pcl::io::OpenNI2Grabber::OpenNI_Default_Mode, pcl::io::OpenNI2Grabber::OpenNI_Default_Mode);
	grabber_->start ();
}

ofApp::~ofApp() {
	delete grabber_;
}

//--------------------------------------------------------------
void ofApp::setup(){
	ofEnableDepthTest();
	//setup gui

	gui = new ofxUICanvas();        //Creates a canvas at (0,0) using the default width
	// storage for moving graph
	vector<float> buffer;
	for(int i = 0; i < 256; i++) {
		buffer.push_back(0.0);
	}
	//fpsLabel =
	gui->addFPS();
	fpsMovingGraph = gui->addMovingGraph("FPS OVER TIME", buffer, 256, 0.0, 120.0);
	gui->addSlider("BACKGROUND",0.0,255.0,100.0);
	gui->addToggle("FULLSCREEN", false);
	gui->addLabel("lDepthTresh","Depth Threshold");
	ofxUISlider * dMaxSl = gui->addSlider("MAXDEPTH", .3, 5.0, 100.0);
	ofxUISlider * dMinSl = gui->addSlider("MINDEPTH", .1, 5.0, 100.0);
	gui->addLabel("lOrgFastMesh","Meshing");
	ofxUISlider * trSl = gui->addSlider("TRIANGLESIZE", 1, 10, 100.0);
	trSl->setIncrement(1);
	
	vector<string> names;
	names.push_back("POINTS");
	names.push_back("WIREFRAME");
	names.push_back("TRIANGLEMESH");
	ofxUIRadio * radio = gui->addRadio("RENDERMODE", names, OFX_UI_ORIENTATION_VERTICAL);

	// scale to fit
	gui->autoSizeToFitWidgets();
	// register listener callback
	ofAddListener(gui->newGUIEvent, this, &ofApp::guiEvent);
	// restore gui state from save file
	gui->loadSettings("settings.xml");

	// set rendermode from gui state
	std::string active = radio->getActiveName();
	if(active == "POINTS"){
		renderMode = RENDER_POINTS;
	}
	else if (active == "WIREFRAME"){
		renderMode = RENDER_WIRE;
	}
	else if (active == "TRIANGLEMESH"){
		renderMode = RENDER_MESH;
	}

	// setup organized fast mesh processor
	ofm.setTriangulationType (pcl::OrganizedFastMesh<PointType>::TRIANGLE_LEFT_CUT);
	ofmPixelSize = trSl->getScaledValue();
	// setup passthrough filter
	depthThreshMax = dMaxSl->getScaledValue();
	depthThreshMin = dMinSl->getScaledValue();
	pass_.setFilterFieldName ("z");
	pass_.setKeepOrganized(true);

	// connect cloud callback to openni grabber
	boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&ofApp::cloud_callback, this, _1);
	boost::signals2::connection cloud_connection = grabber_->registerCallback (cloud_cb);

	// setup mesh
	mesh.enableColors();

	// setup camera
	cam.setPosition(ofVec3f(0, 0, 0));
	cam.lookAt(ofVec3f(0, 0, 4000), ofVec3f(0, 1, 0));
}

//--------------------------------------------------------------
void ofApp::update(){
	// Update FPS Label
	fpsMovingGraph->addPoint(ofGetFrameRate());

	// local store for cloud and vertexIndizes
	boost::shared_ptr<std::vector<pcl::Vertices> > temp_verts(new std::vector<pcl::Vertices>);

	// See if we can get a cloud. If we cant get one because Grabber is writing, we render the last frame again.
	if (cloud_mutex_.try_lock ())
	{
		cloud_.swap (temp_cloud);
		cloud_mutex_.unlock ();
	}

	if(temp_cloud){
		// apply depth Thresholds
		CloudPtr cloudFiltered(new Cloud);
		pass_.setFilterLimits (depthThreshMin, depthThreshMax);
		pass_.setInputCloud(temp_cloud);
		pass_.filter(*cloudFiltered);

		// fast organized mesh triangulation
		ofm.setTrianglePixelSize (ofmPixelSize);
		ofm.setInputCloud(cloudFiltered);
		ofm.reconstruct (*temp_verts);

		// make an ofMesh
		mesh.clear();
		PointType p;

		// triangle mesh
		mesh.setMode(OF_PRIMITIVE_TRIANGLES);
		// the old "crap":
		//for(std::vector<pcl::Vertices>::iterator it = temp_verts->begin(); it != temp_verts->end(); ++it) {
		// The NEW shiny C++11 style :)))
		for(auto &v : *temp_verts) {
			for(auto &pointindex : v.vertices){
				p = cloudFiltered->at(pointindex);
				mesh.addVertex(ofVec3f(-p.x*1000,-p.y*1000,p.z*1000));
				mesh.addColor(ofColor(p.r,p.g,p.b));
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(background);

	cam.begin();
	ofPushMatrix();

	switch(renderMode){
	case RENDER_POINTS:
		mesh.drawVertices();
		break;
	case RENDER_WIRE:
		mesh.drawWireframe();
		break;
	case RENDER_MESH:
		mesh.draw();
		break;
	default:
		mesh.drawVertices();
	}

	ofPopMatrix();
	cam.end();
}

//--------------------------------------------------------------
void ofApp::cloud_callback (const CloudConstPtr& cloud)
{
	FPS_CALC ("cloud callback");
	{
		boost::mutex::scoped_lock lock (cloud_mutex_);
		cloud_ = cloud;
	}
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
	// if mouse on gui
	if(gui->getRect()->getMaxX() >= x && gui->getRect()->getMaxY() >= y){
		cam.disableMouseInput();
	} else {
		cam.enableMouseInput();
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
	gui->saveSettings("settings.xml");
	delete gui;
	grabber_->stop();
}

//--------------------------------------------------------------
void ofApp::guiEvent(ofxUIEventArgs &e)
{
	std::string name = e.getName();
	if(name == "BACKGROUND")
	{
		ofxUISlider *slider = e.getSlider();
		background = slider->getScaledValue();
	}
	else if(name == "FULLSCREEN")
	{
		ofxUIToggle *toggle = e.getToggle();
		ofSetFullscreen(toggle->getValue());
	}
	else if(name == "MAXDEPTH")
	{
		ofxUISlider *slider = e.getSlider();
		depthThreshMax = slider->getScaledValue();
	}
	else if(name == "MINDEPTH")
	{
		ofxUISlider *slider = e.getSlider();
		depthThreshMin = slider->getScaledValue();
	}
	else if(name == "TRIANGLESIZE")
	{
		ofxUISlider *slider = e.getSlider();
		ofmPixelSize = (int)slider->getScaledValue();
	}
	else if(name == "RENDERMODE")
	{
		ofxUIRadio *radio = (ofxUIRadio *) e.widget;
		std::string active = radio->getActiveName();
		if(active == "POINTS"){
			renderMode = RENDER_POINTS;
		}
		else if (active == "WIREFRAME"){
			renderMode = RENDER_WIRE;
		}
		else if (active == "TRIANGLEMESH"){
			renderMode = RENDER_MESH;
		}
	}
}