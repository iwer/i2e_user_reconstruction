#include "ofApp.h"
#include <boost/regex.hpp>

void ofApp::setupUI()
{


	ui_.setup();

	randomRecordingBtn_.addListener(this, &ofApp::invokeLoader);
	ui_.add(randomRecordingBtn_.setup("Next"));
	ui_.add(play_.setup("Play", false));
}

void ofApp::loadRandomPlaySettingData()
{

	Playsettings setting = randomPlaysettings();
	loading_ = true;

	// if only texmap changed, no file loading necessary
	if(setting.take == currentSettings_.take &&
		setting.speed == currentSettings_.speed &&
		setting.quality == currentSettings_.quality)
	{
		currentSettings_ = setting;
		loading_ = false;
		return;
	}
	
	images_.clear();
	meshes_.clear();

	auto basepath = std::string("reconstructed/") +
		take_[setting.take] + std::string("/") +
		speed_[setting.speed] + std::string("/") +
		quality_[setting.quality];

	int framenum = count_files(basepath);
	std::cout << "Path: " << basepath << " " << framenum << std::endl;
	loadState_ = 0.;
	for (auto l = 0; l < framenum; ++l)
	{
		//std::cout << "Loading: " << basepath + std::string("/frame_") + fileNumber(l) + std::string(".ply") << std::endl;
		meshes_[l] = ofMesh();
		meshes_[l].load(basepath + std::string("/frame_") + fileNumber(l) + std::string(".ply"));
		//std::cout << "Loading: " << basepath + std::string("/frame_") + fileNumber(l) + std::string(".png") << std::endl;
		images_[l] = ofImage();
		images_[l].load(basepath + std::string("/frame_") + fileNumber(l) + std::string(".png"));
		images_[l].update();
		loadState_ = (l + 1) * 1.0 / framenum;
		std::cout << "Loading  " << loadState_ * 100 << "%" << std::endl;
		std::flush(std::cout);
	}

	currentSettings_ = setting;
	updateMaxFrames();
	loadState_ = 0; 
	loading_ = false;
}

void ofApp::updateMaxFrames()
{
	auto basepath = std::string("reconstructed/") +
		take_[currentSettings_.take] + std::string("/") +
		speed_[currentSettings_.speed] + std::string("/") +
		quality_[currentSettings_.quality];

	maxFrames_ = count_files(basepath);
}

//--------------------------------------------------------------
void ofApp::setup(){
	font.load("verdana.ttf", 36);
	
	take_.push_back("0");
	take_.push_back("1");
	take_.push_back("2");
	take_.push_back("3");
	take_.push_back("4");
	speed_.push_back("fast");
	//speed_.push_back("slow");
	quality_.push_back("hq");
	//quality_.push_back("lq");

	for(int i = 0; i < 5; i++)
	{
		Playsettings p;
		p.speed = 0;
		p.quality = 0;
		p.take = i;
		p.texmap = false;
		availablePlaysettings_.push_back(p);
		auto p2 = p;
		p2.texmap = true;
		availablePlaysettings_.push_back(p2);
	}
	std::cout << "Play setting available: " << availablePlaysettings_.size() << std::endl;
	setupUI();
	cam_.setFarClip(100000);
	cam_.rotate(180, 0, 1, 0);
	cam_.setDistance(10000);


	frameNum_ = 0;

	invokeLoader();

	ofSetFrameRate(8);
}

//--------------------------------------------------------------
void ofApp::update(){

	if(!loading_)
	{
		try
		{
			loaderthread_->join();
		}
		catch (exception &e)
		{
			
		}
	}
	if(frameNum_ < maxFrames_ - 1)
	{
		if (play_) {
			++frameNum_;
		}
	}
	else
	{
		frameNum_ = 0;
		play_ = false;
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	cam_.enableMouseInput();
	cam_.disableMouseMiddleButton();

	ofBackground(127);
	
	auto text = std::to_string(currentSettings_.take) + std::to_string(currentSettings_.speed) + std::to_string(currentSettings_.quality) + std::to_string(currentSettings_.texmap);

	font.drawString(text, ofGetWidth() - 140, 50);
	if (!loading_ && images_[frameNum_].isAllocated()) {
		images_[frameNum_].draw(ofGetWidth() / 4 * 3, ofGetHeight() / 4 * 3, ofGetWidth() / 4, ofGetHeight() / 4);
	}
	

	ofEnableDepthTest();

	if (loadState_ > 0)
	{
		ofPath p;
		p.arc(ofGetWidth() - 80, 120, 40, 40, 0, loadState_ * 360);
		p.draw();
		ofNoFill();
		ofDrawCircle(ofGetWidth() - 80, 120, 0, 40);
	}


	cam_.begin();
	
	ofDrawAxis(1000);
	
	if (!loading_) {
		if (currentSettings_.texmap) {
			if (images_[frameNum_].isAllocated()) {
				images_[frameNum_].getTexture().bind();
				meshes_[frameNum_].disableColors();
				meshes_[frameNum_].enableTextures();
				meshes_[frameNum_].draw();
				images_[frameNum_].getTexture().unbind();
			}
		}
		else
		{
			meshes_[frameNum_].enableColors();
			meshes_[frameNum_].disableTextures();
			meshes_[frameNum_].draw();
		}
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

std::string ofApp::fileNumber(int number) {
	std::ostringstream ss;
	ss << std::setw(5) << std::setfill('0') << number;
	return ss.str();
}

void ofApp::invokeLoader()
{
	if (!loading_) {
		loaderthread_ = new std::thread(&ofApp::loadRandomPlaySettingData, this);
		loaderthread_->detach();
	}
}

int ofApp::count_files(std::string basepath)
{
	boost::filesystem::path Path(std::string("data/") + basepath);
	boost::regex exp_ply(std::string("data/") + basepath + std::string("/frame_") + std::string("[0-9]{5}.ply"));
	boost::regex exp_png(std::string("data/") + basepath + std::string("/frame_") + std::string("[0-9]{5}.png"));
	int numberPlyFiles = 0;
	int numberPngFiles = 0;
	boost::filesystem::directory_iterator end_iter; // Default constructor for an iterator is the end iterator

	if (boost::filesystem::is_directory(Path)) {
		for (boost::filesystem::directory_iterator iter(Path); iter != end_iter; ++iter) {
			if (boost::regex_match(iter->path().generic_string(), exp_ply)) {
				++numberPlyFiles;
			}
			if (boost::regex_match(iter->path().generic_string(), exp_png)) {
				++numberPngFiles;
			}
		}
	} else
	{
		std::cerr << "No directory: " << basepath << std::endl;
	}

	if (numberPlyFiles != numberPngFiles)
	{
		std::cout << "WARNING: Number of .ply (" << numberPlyFiles << ") and .png (" << numberPngFiles << ") files do not match." << std::endl;
	}


	return std::min(numberPlyFiles, numberPngFiles);
}

ofApp::Playsettings ofApp::randomPlaysettings()
{
	std::srand(std::time(nullptr));
	int i = std::rand() % availablePlaysettings_.size();
	auto s = availablePlaysettings_.at(i);
	std::cout << s.take << ":" << s.speed << ":" << s.quality << ":" << s.texmap << std::endl;
	return  s;
}
