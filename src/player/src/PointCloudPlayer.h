#pragma once

#include <string>
#include <thread>
#include <chrono>
#include <boost/signals2.hpp>
#include "recon/typedefs.h"

class PointCloudPlayer
{
public:
	PointCloudPlayer();
	~PointCloudPlayer();

	void setBasePath(std::string path);
	void setFramesPerSecond(int fps);
	int getNumberFrames();

	void start();
	void stop();

	boost::signals2::signal<void (int, recon::CloudPtr)> callback;
private:
	bool running_;
	int fps_;
	int readIndex_;
	int numberOfFiles_;

	std::chrono::duration<long long, std::nano> frameTime_;
	std::chrono::duration<long long, std::nano> avgReadFromDiskTime_;
	std::string basepath_;

	std::thread * read_thread_;

	std::string fileNumber();
	int count_files(std::string directory, std::string ext);
	void readThreadFunction();
};

