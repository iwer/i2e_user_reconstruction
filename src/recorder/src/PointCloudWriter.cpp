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
	if (running_) {
		stop();
	}
}

void PointCloudWriter::setBaseFileName(std::string & filename)
{
	base_filename_ = filename;
}


void PointCloudWriter::enquePointcloudForWriting(int sensorId, recon::CloudConstPtr cloud)
{
	std::lock_guard<std::mutex> lock(queue_lock_);
	cloud_queue_.push(std::pair<int, recon::CloudConstPtr>(sensorId, cloud));
}

void PointCloudWriter::start()
{
	running_ = true;
	write_thread_ = new std::thread(&PointCloudWriter::writeThreadFunction, this);
}

void PointCloudWriter::stop()
{
	running_ = false;
	write_thread_->join();
}

int PointCloudWriter::getQueueLength()
{
	std::lock_guard<std::mutex> lock(queue_lock_);
	return cloud_queue_.size();
}

std::string PointCloudWriter::fileNumber() {
	std::ostringstream ss;
	ss << std::setw(5) << std::setfill('0') << writeIndex_;
	return ss.str();
}

void PointCloudWriter::writeThreadFunction()
{
	std::cout << "Starting writeThread" << std::endl;
	while (running_) {
		if (!cloud_queue_.empty()) {
			std::lock_guard<std::mutex> lock(queue_lock_);
			if (!cloud_queue_.empty()) {
				int id = cloud_queue_.front().first;
				recon::CloudConstPtr c = cloud_queue_.front().second;

				std::string name = base_filename_ + std::string("_s")
					+ std::to_string(id) + std::string("_") + fileNumber() + std::string(".pcd");
				std::cout << "Writing: " << name << std::endl;


				cloud_queue_.pop();
				if (c->size() > 0) {
					pcl::io::savePCDFileBinary(name, *c.get());
				}
				++writeIndex_;
			}
			else {
				std::this_thread::sleep_for(30ms);
			}
		}
	}
}
