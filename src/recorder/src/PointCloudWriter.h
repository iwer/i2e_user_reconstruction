#pragma once
#include <queue>
#include <thread>
#include <mutex>
#include <string>
#include "recon/typedefs.h"
#include "recon/AbstractSensor.h"
#include <pcl/io/file_io.h>

class PointCloudWriter
{
public:
	PointCloudWriter();
	~PointCloudWriter();

	void setBaseFileName(std::string & filename);
	void setSensorDetails(recon::AbstractSensor::Ptr sensor);
	void enquePointcloudForWriting(recon::CloudConstPtr cloud);

	void start();
	void stop();

	int getQueueLength();
private:
	bool running_;
	int sensor_ID_;
	Eigen::Vector4f sensor_translation_;
	Eigen::Quaternionf sensor_rotation_;

	int writeIndex_;
	std::string base_filename_;
	std::queue<recon::CloudConstPtr> cloud_queue_;
	std::mutex queue_lock_;

	pcl::PCDWriter file_writer_;

	void writeThreadFunction();
	std::string fileNumber();
};

