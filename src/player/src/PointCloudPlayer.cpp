#include "PointCloudPlayer.h"
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/regex.hpp>
#include <pcl/io/image_rgb24.h>
#include <pcl/io/lzf_image_io.h>

PointCloudPlayer::PointCloudPlayer()
	: running_(false)
	, readIndex_(0)
	, sensor_index_(0)
{
	setFramesPerSecond(1);
}

PointCloudPlayer::PointCloudPlayer(std::string path, int index, int fps)
	: running_(false)
	, readIndex_(0)
	, sensor_index_(index)
	, basepath_(path)
{
	setFramesPerSecond(fps);
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
	boost::regex e1(basepath_ + std::string("/capture_s") + std::to_string(sensor_index_) + std::string("_[0-9]{5}.pcd"));
	int numberFileswithExt = 0;
	boost::filesystem::directory_iterator end_iter; // Default constructor for an iterator is the end iterator

	if (boost::filesystem::is_directory(Path)) {
		for (boost::filesystem::directory_iterator iter(Path); iter != end_iter; ++iter) {
			//if (iter->path().extension() == ext)
			std::cout << iter->path().generic_string() << std::endl;
			if (boost::regex_match(iter->path().generic_string(), e1))
				++numberFileswithExt;
			
		}
	}
	std::cout << "Files for sensor " << sensor_index_ << " : " << numberFileswithExt;
	return numberFileswithExt;
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
	boost::regex e1(path + std::string("/capture_s") + std::string("(\\d+)_[0-9]{5}.pcd"));
	boost::match_results<boost::filesystem::directory_iterator> what;
	boost::filesystem::directory_iterator end_iter; // Default constructor for an iterator is the end iterator
	
	if (boost::filesystem::is_directory(Path)) {
		boost::filesystem::directory_iterator iter(Path);
		//boost::regex_search(iter, end_iter, what, e1, boost::match_default);

		//std::cout << what.length() << std::endl;
		//for (auto &r : what)
		//{
		//	std::cout << r << std::endl;
		//}
		
	}
	return 0;
}

int PointCloudPlayer::getNumberFrames()
{
	return numberOfFiles_;
}

void PointCloudPlayer::start()
{
	running_ = true;
	read_thread_ = new std::thread(&PointCloudPlayer::readThreadFunction, this);
}

void PointCloudPlayer::stop()
{
	running_ = false;
	read_thread_->join();
}

std::string PointCloudPlayer::fileNumber() {
	std::ostringstream ss;
	ss << std::setw(5) << std::setfill('0') << readIndex_;
	return ss.str();
}

void PointCloudPlayer::readThreadFunction()
{
	while (running_) {
		// assemble file path string
		std::string filePath(basepath_ + std::string("/capture_s") + std::to_string(sensor_index_)
			+ std::string("_") + fileNumber() + std::string(".pcd"));
		// if filePath is a file, try to open as .pcd
		if (boost::filesystem::is_regular_file(boost::filesystem::path(filePath))) {
			recon::Cloud cloud;
			auto start = std::chrono::high_resolution_clock::now();
			int cerr = pcl::io::loadPCDFile(filePath, cloud);
			auto end = std::chrono::high_resolution_clock::now();
			auto elapsed_time = end - start;
			avgReadFromDiskTime_ = avgReadFromDiskTime_ + ((elapsed_time - avgReadFromDiskTime_) / (readIndex_ + 1));
			
			//pcl::io::ImageRGB24 image;
			//pcl::io::LZFRGB24ImageReader reader;

			//int ierr = reader.

			
			
			if (!cerr) {
				if (cloud.size() > 0) {
					recon::CloudPtr cloudPtr = boost::make_shared<recon::Cloud>(cloud);
					callback(readIndex_, sensor_index_, cloudPtr);

					auto sleeptime = frameTime_ - avgReadFromDiskTime_;
					std::this_thread::sleep_for(sleeptime);
					++readIndex_;

					if (readIndex_ >= numberOfFiles_) {
						readIndex_ = 0;
					}
				}
			}
		}
	}
}
