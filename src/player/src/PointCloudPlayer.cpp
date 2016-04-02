#include "PointCloudPlayer.h"
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>


PointCloudPlayer::PointCloudPlayer()
	: running_(false)
	, readIndex_(0)
{
	setFramesPerSecond(1);
}


PointCloudPlayer::~PointCloudPlayer()
{
}

void PointCloudPlayer::setBasePath(std::string path)
{
	basepath_ = path;

	//if (boost::filesystem::is_directory(basepath_)) {
	//	std::cout << basepath_ << " is a directory containing:\n";
	//	for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(basepath_), {}))
	//		std::cout << entry << "\n";
	//}

	numberOfFiles_ = count_files(basepath_, std::string(".pcd"));
}

int PointCloudPlayer::count_files(std::string directory, std::string ext)
{
	namespace fs = boost::filesystem;
	boost::filesystem::path Path(directory);
	int Nb_ext = 0;
	boost::filesystem::directory_iterator end_iter; // Default constructor for an iterator is the end iterator

	for (boost::filesystem::directory_iterator iter(Path); iter != end_iter; ++iter)
		if (iter->path().extension() == ext)
			++Nb_ext;

	return Nb_ext;
}

void PointCloudPlayer::setFramesPerSecond(int fps)
{
	fps_ = fps;
	std::chrono::seconds sec(1);
	frameTime_ = sec / fps_;
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
		recon::Cloud cloud;
		auto start = std::chrono::high_resolution_clock::now();
		int err = pcl::io::loadPCDFile(basepath_ + std::string("/captures0_") + fileNumber() + std::string(".pcd"), cloud);
		auto end = std::chrono::high_resolution_clock::now();
		auto elapsed_time = end - start;
		avgReadFromDiskTime_ = avgReadFromDiskTime_ + ((elapsed_time - avgReadFromDiskTime_) / (readIndex_ + 1));
		if (!err) {
			if (cloud.size() > 0) {
				recon::CloudPtr cloudPtr = boost::make_shared<recon::Cloud>(cloud);
				callback(readIndex_, cloudPtr);
				
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
