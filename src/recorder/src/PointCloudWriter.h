#pragma once
#include <queue>
#include <thread>
#include <mutex>
#include <string>
#include "recon/typedefs.h"
#include "recon/AbstractSensor.h"

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
	class SaveTriplet
	{
	public:
		SaveTriplet(int id, recon::CloudConstPtr cloud, recon::ImagePtr image)
			: id_(id)
			, cloud_(cloud)
			, image_(image)
		{}
		int id_;
		recon::CloudConstPtr cloud_;
		recon::ImagePtr image_;
	};

	bool running_;
	Eigen::Vector4f sensor_translation_;
	Eigen::Quaternionf sensor_rotation_;

	int writeIndex_;
	std::string base_filename_;

	std::queue<SaveTriplet> queue_;

	std::mutex queue_lock_;

	std::thread * write_thread_;

	void writeThreadFunction();
	std::string fileNumber();


};

