#include "ofApp.h"
#include <boost/regex.hpp>

void ofApp::setupUI()
{


	ui_.setup();
	ui_.add(takeLbl_.setup("Take", take_[takeIdx_]));
	nextTakeBtn_.addListener(this, &ofApp::nextTake);
	prevTakeBtn_.addListener(this, &ofApp::prevTake);
	ui_.add(nextTakeBtn_.setup("Next Take"));
	ui_.add(prevTakeBtn_.setup("Previous Take"));
	ui_.add(speedLbl_.setup("Speed", speed_[speedIdx_]));
	nextSpeedBtn_.addListener(this, &ofApp::nextSpeed);
	prevSpeedBtn_.addListener(this, &ofApp::prevSpeed);
	ui_.add(nextSpeedBtn_.setup("Next Speed"));
	ui_.add(prevSpeedBtn_.setup("Previous Speed"));
	ui_.add(qualityLbl_.setup("Quality", quality_[qualityIdx_]));
	nextQualityBtn_.addListener(this, &ofApp::nextQuality);
	prevQualityBtn_.addListener(this, &ofApp::prevQuality);
	ui_.add(nextQualityBtn_.setup("Next Quality"));
	ui_.add(prevQualityBtn_.setup("Previous Quality"));
}

void ofApp::loadData()
{
	for (auto i = 0; i < take_.size(); ++i)
	{
		for (auto j = 0; j < speed_.size(); ++j)
		{
			for (auto k = 0; k < quality_.size(); ++k)
			{
				auto basepath = std::string("reconstructed/") +
					take_[i] + std::string("/") +
					speed_[j] + std::string("/") +
					quality_[k] ;
				int framenum = count_files(basepath);
				std::cout << "Path: " << basepath << " " << framenum << std::endl;
				for (auto l = 0; l < framenum; ++l)
				{
					std::cout << "Loading: " << basepath + std::string("/frame_") + fileNumber(l) + std::string(".ply") << std::endl;
					meshes_[i][j][k][l] = ofMesh();
					meshes_[i][j][k][l].load(basepath + std::string("/frame_") + fileNumber(l) + std::string(".ply"));
					std::cout << "Loading: " << basepath + std::string("/frame_") + fileNumber(l) + std::string(".png") << std::endl;
					images_[i][j][k][l] = ofImage();
					images_[i][j][k][l].load(basepath + std::string("/frame_") + fileNumber(l) + std::string(".png"));
					images_[i][j][k][l].update();
				}
			}
		}
	}
}

void ofApp::updateMaxFrames()
{
	auto basepath = std::string("reconstructed/") +
		take_[takeIdx_] + std::string("/") +
		speed_[speedIdx_] + std::string("/") +
		quality_[qualityIdx_];

	maxFrames_ = count_files(basepath);
}

//--------------------------------------------------------------
void ofApp::setup(){
	takeIdx_ = speedIdx_ = qualityIdx_ = 0;

	take_.push_back("first");
	speed_.push_back("fast");
	speed_.push_back("slow");
	quality_.push_back("hq");
	quality_.push_back("lq");

	setupUI();

	cam_.setFarClip(100000);
	cam_.rotate(180, 0, 1, 0);
	cam_.enableMouseInput();
	cam_.disableMouseMiddleButton();

	frameNum_ = 0;

	loadData();

	updateMaxFrames();
	ofSetFrameRate(10);
}

//--------------------------------------------------------------
void ofApp::update(){
	if(frameNum_ < maxFrames_ - 1)
	{
		++frameNum_;
	}
	else
	{
		frameNum_ = 0;
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(127);
	

	images_[takeIdx_][speedIdx_][qualityIdx_][frameNum_].draw(ofGetWidth() / 4 * 3, ofGetHeight() / 4 * 3, ofGetWidth() / 4, ofGetHeight() / 4);

	cam_.begin();
	
	ofEnableDepthTest();
	ofDrawAxis(1000);
	
	
	images_[takeIdx_][speedIdx_][qualityIdx_][frameNum_].getTexture().bind();
	meshes_[takeIdx_][speedIdx_][qualityIdx_][frameNum_].disableColors();
	meshes_[takeIdx_][speedIdx_][qualityIdx_][frameNum_].enableTextures();
	meshes_[takeIdx_][speedIdx_][qualityIdx_][frameNum_].draw();
	images_[takeIdx_][speedIdx_][qualityIdx_][frameNum_].getTexture().unbind();
	
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

void ofApp::nextTake()
{
	if(takeIdx_ < take_.size() - 1)
	{
		++takeIdx_;
	}

	takeLbl_ = take_[takeIdx_];
	updateMaxFrames();
}

void ofApp::prevTake()
{
	if (takeIdx_ > 0)
	{
		--takeIdx_;
	}

	takeLbl_ = take_[takeIdx_];
	updateMaxFrames();
}

void ofApp::nextSpeed()
{
	if (speedIdx_ < speed_.size() - 1)
	{
		++speedIdx_;
	}

	speedLbl_ = speed_[speedIdx_];
	updateMaxFrames();
}

void ofApp::prevSpeed()
{
	if (speedIdx_ > 0)
	{
		--speedIdx_;
	}

	speedLbl_ = speed_[speedIdx_];
	updateMaxFrames();
}

void ofApp::nextQuality()
{
	if (qualityIdx_ < quality_.size() - 1)
	{
		++qualityIdx_;
	}

	qualityLbl_ = quality_[qualityIdx_];
	updateMaxFrames();
}

void ofApp::prevQuality()
{
	if (qualityIdx_ > 0)
	{
		--qualityIdx_;
	}

	qualityLbl_ = quality_[qualityIdx_];
	updateMaxFrames();
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