#pragma once

#include <string>
#include <thread>
#include <chrono>
#include <boost/signals2.hpp>
#include "recon/typedefs.h"
#include <ofImage.h>
#include "PclCloudAndOfImage.h"

class PointCloudPlayer
{
public:
	typedef boost::shared_ptr<PointCloudPlayer> Ptr;

	PointCloudPlayer();
	PointCloudPlayer(std::string path, int index, int fps);
	~PointCloudPlayer();

	void setBasePath(std::string path);
	void setSensorIndex(int index);
	void setFramesPerSecond(int fps);
	static int getNumberSensors(std::string path);
	int getNumberFrames();
	void setFrameNumber(int number);

	PclCloudAndOfImage::Ptr requestFrame();
	PclCloudAndOfImage::Ptr requestFrame(int number);


	void start();
	void stop();

	boost::signals2::signal<void (int, int, recon::CloudPtr, std::shared_ptr<ofImage>)> callback;
private:
	bool running_;
	bool looping_;
	int fps_;

	int sensor_index_;
	int readIndex_;
	int numberOfFiles_;

	std::chrono::duration<long long, std::nano> frameTime_;
	std::chrono::duration<long long, std::nano> avgReadFromDiskTime_;
	std::string basepath_;

	std::thread * read_thread_;

	std::string fileNumber(int number);
	int count_files();
	void readThreadFunction();
};

