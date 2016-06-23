#pragma once
#include <queue>
#include <thread>
#include <mutex>
#include <string>
#include "recon/typedefs.h"
#include "recon/AbstractSensor.h"
#include <common/PclCloudAndImage.h>

class PointCloudWriter
{
public:
	PointCloudWriter();
	~PointCloudWriter();

	void setBaseFileName(std::string & filename);
	void enquePointcloudForWriting(int sensorId, recon::CloudConstPtr cloud, recon::ImagePtr image);

	void start();
	void stop();

	int getQueueLength();
private:
	bool running_;
	Eigen::Vector4f sensor_translation_;
	Eigen::Quaternionf sensor_rotation_;

	std::map<int,int> writeIndex_;
	std::string base_filename_;

	std::queue<PclCloudAndImage> queue_;

	std::mutex queue_lock_;

	std::thread * write_thread_;

	void writeThreadFunction();
	std::string fileNumber(int number);


};

