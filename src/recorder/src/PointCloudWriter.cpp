#include "PointCloudWriter.h"
#include <chrono>

using namespace std::chrono_literals;

PointCloudWriter::PointCloudWriter() 
	: running_(false)
	, writeIndex_(0)
{
}


PointCloudWriter::~PointCloudWriter()
{
}

void PointCloudWriter::setBaseFileName(std::string & filename)
{
	base_filename_ = filename;
}

void PointCloudWriter::setSensorDetails(recon::AbstractSensor::Ptr sensor)
{
	sensor_ID_ = sensor->getId();
}

void PointCloudWriter::enquePointcloudForWriting(recon::CloudConstPtr cloud)
{
	//std::lock_guard<std::mutex> lock(queue_lock_);
	cloud_queue_.push(cloud);
}

void PointCloudWriter::start()
{
	running_ = true;
	std::thread write_thread_(&PointCloudWriter::writeThreadFunction, this);
}

void PointCloudWriter::stop()
{
	running_ = false;
}

int PointCloudWriter::getQueueLength()
{
	return cloud_queue_.size();
}

std::string PointCloudWriter::fileNumber() {
	std::ostringstream ss;
	ss << std::setw(5) << std::setfill('0') << writeIndex_;
	return ss.str();
}

void PointCloudWriter::writeThreadFunction()
{
	while (running_) {
		if (!cloud_queue_.empty()) {
			//std::lock_guard<std::mutex> lock(queue_lock_);
			if (!cloud_queue_.empty()) {
				std::string name = base_filename_ + std::string("s")
					+ std::to_string(sensor_ID_) + std::string("_") + fileNumber();
				std::cout << "Writing: " << name << std::endl;

				recon::CloudConstPtr c = cloud_queue_.front();
				cloud_queue_.pop();
				file_writer_.writeBinary(name, *c.get());
				++writeIndex_;
			}
			else {
				std::this_thread::sleep_for(10ms);
			}
		}
	}
}
