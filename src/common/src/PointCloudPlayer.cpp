#include "common/PointCloudPlayer.h"
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/regex.hpp>
#include <pcl/io/image_rgb24.h>
#include <pcl/io/lzf_image_io.h>
#include <set>

PointCloudPlayer::PointCloudPlayer()
	: running_(false)
	, looping_(true)
	, readIndex_(0)
	, sensor_index_(0)
{
	setFramesPerSecond(1);
	numberOfFiles_ = count_files();
}

PointCloudPlayer::PointCloudPlayer(std::string path, int index, int fps)
	: running_(false)
	, looping_(true)
	, readIndex_(0)
	, sensor_index_(index)
	, basepath_(path)
{
	setFramesPerSecond(fps);
	numberOfFiles_ = count_files();
}

PointCloudPlayer::~PointCloudPlayer()
{
}

void PointCloudPlayer::setBasePath(std::string path)
{
	basepath_ = path;
	numberOfFiles_ = count_files();
}

void PointCloudPlayer::setSensorIndex(int index)
{
	sensor_index_ = index;
	numberOfFiles_ = count_files();
}

int PointCloudPlayer::count_files()
{
	std::string ext(".pcd");
	boost::filesystem::path Path(basepath_);
	boost::regex exp_pcd(basepath_ + std::string("/capture_s") + std::to_string(sensor_index_) + std::string("_[0-9]{5}.pcd"));
	boost::regex exp_jpg(basepath_ + std::string("/capture_s") + std::to_string(sensor_index_) + std::string("_[0-9]{5}.jpg"));
	int numberPcdFiles = 0;
	int numberJpgFiles = 0;
	boost::filesystem::directory_iterator end_iter; // Default constructor for an iterator is the end iterator

	if (boost::filesystem::is_directory(Path)) {
		for (boost::filesystem::directory_iterator iter(Path); iter != end_iter; ++iter) {
			if (boost::regex_match(iter->path().generic_string(), exp_pcd)) {
				++numberPcdFiles;
			}
			if (boost::regex_match(iter->path().generic_string(), exp_jpg)) {
				++numberJpgFiles;
			}
		}
	}
	
	if(numberPcdFiles != numberJpgFiles)
	{
		std::cout << "WARNING: Number of .pcd ("<< numberPcdFiles << ") and .jpg (" << numberJpgFiles << ") files do not match." << std::endl;
	}
	std::cout << "Files for sensor " << sensor_index_ << " : " << numberPcdFiles << std::endl;
	
	
	return std::min(numberPcdFiles, numberJpgFiles);
}

void PointCloudPlayer::setFramesPerSecond(int fps)
{
	fps_ = fps;
	std::chrono::milliseconds sec(1000);
	frameTime_ = sec / fps_;
}

int PointCloudPlayer::getNumberSensors(std::string path)
{
	boost::filesystem::path Path(path);
	boost::regex e1(std::string("capture_s") + std::string("(\\d+)_[0-9]{5}.pcd"));
	boost::match_results<std::string::const_iterator> what;
	boost::filesystem::directory_iterator end_iter; // Default constructor for an iterator is the end iterator
	
	std::set<std::string> sensorIndizes;

	if (boost::filesystem::is_directory(Path)) {
		boost::filesystem::directory_iterator iter(Path);
		
		for (auto it = iter; it != end_iter; ++it) {

			if (boost::filesystem::is_regular_file(*it)) {

				if (boost::regex_search(it->path().filename().string(), what, e1, boost::match_default))
				{
					std::string res = what[1];
					//std::cout << res << std::endl;
					sensorIndizes.insert(res);
				}

			} else
			{
				std::cerr << " Not a regular file: " << *it << std::endl;
			}
		}
	}else
	{
		std::cerr << "Not a valid path: " << Path.string() << std::endl;
	}
	return sensorIndizes.size();
}

int PointCloudPlayer::getNumberFrames()
{
	return numberOfFiles_;
}

void PointCloudPlayer::setFrameNumber(int number)
{
	if(number >= 0 && number < numberOfFiles_)
	{
		readIndex_ = number;
	} else
	{
		std::cout << "Frame number not in range" << std::endl;
	}
}

PclCloudAndOfImage::Ptr PointCloudPlayer::requestFrame()
{
	return requestFrame(readIndex_);
}

PclCloudAndOfImage::Ptr PointCloudPlayer::requestFrame(int number)
{
	PclCloudAndOfImage::Ptr p;
	if(!running_)
	{
		std::shared_ptr<ofImage> image(new ofImage());
		recon::Cloud cloud;

		auto filePath(basepath_ + std::string("/capture_s") + std::to_string(sensor_index_)
			+ std::string("_") + fileNumber(number) + std::string(".pcd"));
		auto imagePath(basepath_ + std::string("/capture_s") + std::to_string(sensor_index_)
			+ std::string("_") + fileNumber(number) + std::string(".jpg"));

		int cerr = pcl::io::loadPCDFile(filePath, cloud);
		auto img_loaded = image->load(imagePath);
		if (!cerr) {
			if (cloud.size() > 0 && img_loaded) {
				p.reset(new PclCloudAndOfImage(sensor_index_, cloud.makeShared(), image));
			}
		}
	} else
	{
		std::cout << "PointCloudPlayer is running in async mode (e.g. started), connect to ::callback to receive frames." << std::endl;
	}
	return p;
}

void PointCloudPlayer::start()
{
	std::cout << "Player " << sensor_index_ << " starting" << std::endl;
	running_ = true;
	read_thread_ = new std::thread(&PointCloudPlayer::readThreadFunction, this);
}

void PointCloudPlayer::stop()
{
	std::cout << "Player " << sensor_index_ << " stopping" << std::endl;
	running_ = false;
	read_thread_->join();
}

std::string PointCloudPlayer::fileNumber(int number) {
	std::ostringstream ss;
	ss << std::setw(5) << std::setfill('0') << number;
	return ss.str();
}

void PointCloudPlayer::readThreadFunction()
{
	while (running_) {
		// assemble file path string
		std::string filePath(basepath_ + std::string("/capture_s") + std::to_string(sensor_index_)
			+ std::string("_") + fileNumber(readIndex_) + std::string(".pcd"));
		std::string imagePath(/*std::string("data/recorder/capture_s")*/ basepath_ + std::string("/capture_s") + std::to_string(sensor_index_)
			+ std::string("_") + fileNumber(readIndex_) + std::string(".jpg"));
		// if filePath is a file, try to open as .pcd
		if (boost::filesystem::is_regular_file(boost::filesystem::path(filePath))) {
			recon::Cloud cloud;
			std::shared_ptr<ofImage> image(new ofImage());

			auto start = std::chrono::high_resolution_clock::now();

			int cerr = pcl::io::loadPCDFile(filePath, cloud);
			auto img_loaded = image->load(imagePath);

			auto end = std::chrono::high_resolution_clock::now();
			auto elapsed_time = end - start;
			avgReadFromDiskTime_ = avgReadFromDiskTime_ + ((elapsed_time - avgReadFromDiskTime_) / (readIndex_ + 1));
			

			//std::cout << "Loaded image: " << imagePath << std::endl << image->getWidth() << "x" << image->getHeight() << std::endl;
			if (!cerr) {
				if (cloud.size() > 0 && img_loaded) {
					// call application
					callback(readIndex_, sensor_index_, cloud.makeShared(), image);
				}
			}


			readIndex_++;

			// reset framenumber at the end
			if (readIndex_ + 1 >= numberOfFiles_) {
				readIndex_ = 0;
			}

			// sleep
			auto sleeptime = frameTime_ - avgReadFromDiskTime_;
			std::this_thread::sleep_for(sleeptime);

		}
	}
}
