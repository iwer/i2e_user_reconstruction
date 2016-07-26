#include "ofApp.h"
#include "PlaySettings.h"
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
	loading_ = true;

	auto setting = session_.at(settingsIdx_);
	if(setting.manual_)
	{
		if (settingsIdx_ < session_.size() - 1) {
			++settingsIdx_;
			std::cout << "Pre inc: " << settingsIdx_ << std::endl;
			setting = session_.at(settingsIdx_);
		} else
		{
			testDone_ = true;
			return;
		}
	}


	std::cout << setting.take_ << ":" << setting.speed_ << ":" << setting.quality_ << ":" << setting.texmap_ << std::endl;

	images_.clear();
	meshes_.clear();

	auto basepath = std::string("reconstructed/") +
		take_[setting.take_] + std::string("/") +
		speed_[setting.speed_] + std::string("/") +
		quality_[setting.quality_];

	auto framenum = count_files(basepath);
	std::cout << "Path: " << basepath << " " << framenum << std::endl;
	loadState_ = 0.;
	for (auto l = 0; l < framenum; ++l)
	{
		//std::cout << "Loading: " << basepath + std::string("/frame_") + fileNumber(l) + std::string(".ply") << std::endl;
		meshes_[l] = ofMesh();
		meshes_[l].load(basepath + std::string("/frame_") + fileNumber(l) + std::string(".ply"));
		//std::cout << "Loading: " << basepath + std::string("/frame_") + fileNumber(l) + std::string(".png") << std::endl;
		images_[l] = ofImage();
		images_[l].setUseTexture(false);
		images_[l].load(basepath + std::string("/frame_") + fileNumber(l) + std::string(".png"));
		//ofLoadImage(pixelData_[l], basepath + std::string("/frame_") + fileNumber(l) + std::string(".png"));
		loadState_ = (l + 1) * 1.0 / framenum;
		//std::cout << "Loading  " << loadState_ * 100 << "%" << std::endl;
		//std::flush(std::cout);
	}

	currentSettings_ = setting;
	updateMaxFrames();
	loadState_ = 0; 

	if(settingsIdx_ < session_.size())
	{
		++settingsIdx_;
		std::cout << "Post inc: " << settingsIdx_ << std::endl;
	}
	loading_ = false;
	textureReloadNeeded_ = true;
}

void ofApp::updateMaxFrames()
{
	auto basepath = std::string("reconstructed/") +
		take_[currentSettings_.take_] + std::string("/") +
		speed_[currentSettings_.speed_] + std::string("/") +
		quality_[currentSettings_.quality_];

	maxFrames_ = count_files(basepath);
}

//--------------------------------------------------------------
void ofApp::setup(){
	font.load("verdana.ttf", 36);
	smallfont.load("verdana.ttf", 24);
	hugefont.load("verdana.ttf", 56);

	take_.push_back("0");
	take_.push_back("1");
	take_.push_back("2");
	take_.push_back("3");
	take_.push_back("4");
	speed_.push_back("fast");
	speed_.push_back("slow");
	quality_.push_back("hq");
	quality_.push_back("lq");

	//take
	for(auto i = 0; i < 5; i++)
	{
		//speed
		for (auto j = 0; j < 2; j++) {
			//quality
			for (auto k = 0; k < 2; k++) {
				//texmap
				for (auto l = 0; l < 2; l++) {
					PlaySettings p(i, j, k, (l == 0) ? false : true);
					PlaySettings::availablePlaysettings_.push_back(p);
				}
			}
		}
	}
	std::cout << "Play setting available: " << PlaySettings::availablePlaysettings_.size() << std::endl;

	session_ = PlaySettings::generateTestSequence();
	std::cout << "Session size: " << session_.size() << std::endl;

	setupUI();
	cam_.setFarClip(100000);
	cam_.rotate(180, 0, 1, 0);
	cam_.setDistance(3000);

	settingsIdx_ = 0;
	frameNum_ = 0;
	testDone_ = false;

	invokeLoader();
	textureReloadNeeded_ = false;
	ofSetFrameRate(6);
}

//--------------------------------------------------------------
void ofApp::update(){
	// kill loaderthread if finished loading
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

	// update images in gpu memory
	if (textureReloadNeeded_) {
		for (auto &i : images_)
		{
			i.second.setUseTexture(true);
			i.second.update();
		}
		textureReloadNeeded_ = false;
	}

	// progress framenumber
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

std::string ofApp::settingsString(PlaySettings &set)
{
	return std::to_string(set.take_) + std::to_string(set.speed_) + std::to_string(set.quality_) + std::to_string(set.texmap_);
}

//--------------------------------------------------------------
void ofApp::draw(){
	cam_.enableMouseInput();
	cam_.disableMouseMiddleButton();

	ofBackground(127);
	if (!testDone_) {
		auto text = settingsString(currentSettings_);

		ofSetColor(255, 255, 255);
		font.drawString(text, ofGetWidth() - 140, 50);

		for (auto i = 0; i < session_.size(); i++)
		{
			auto t = settingsString(session_.at(i));
			if (session_.at(i).manual_)
			{
				ofSetColor(255, 0, 0);
			}
			else
			{
				ofSetColor(255, 255, 255);
			}
			smallfont.drawString(t, ofGetWidth() - 120, 200 + i * 28);
		}


		ofEnableDepthTest();
		ofSetColor(255, 255, 255);

		if (loadState_ > 0)
		{
			ofPath p;
			ofSetColor(255, 255, 255);
			p.arc(ofGetWidth() - 80, 120, 40, 40, 0, loadState_ * 360);
			p.draw();
			ofNoFill();
			ofDrawCircle(ofGetWidth() - 80, 120, 0, 40);
		}


		cam_.begin();

		ofDrawAxis(300);

		if (!loading_) {
			if (currentSettings_.texmap_) {
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
	} else
	{
		hugefont.drawString("Vielen Dank!", ofGetWidth() / 2 - 250, ofGetHeight() / 2 - 28);
	}
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
	if(settingsIdx_ >= session_.size() - 1)
	{
		images_.clear();
		meshes_.clear();
		testDone_ = true;
	}
	else {
		if (!loading_) {
			loaderthread_ = new std::thread(&ofApp::loadRandomPlaySettingData, this);
			loaderthread_->detach();
		}
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


