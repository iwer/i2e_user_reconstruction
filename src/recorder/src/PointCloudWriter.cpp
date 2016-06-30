#include "PointCloudWriter.h"
#include <chrono>
#include <pcl/io/file_io.h>
#include <ofTexture.h>
#include <of-pcl-bridge/of-pcl-bridge.h>

using namespace std::chrono_literals;

PointCloudWriter::PointCloudWriter() 
	: running_(false)
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


void PointCloudWriter::enquePointcloudForWriting(int sensorId, recon::CloudConstPtr cloud, recon::ImagePtr image)
{
	std::lock_guard<std::mutex> lock(queue_lock_);
	queue_.push(PclCloudAndImage(sensorId, cloud, image));

	try
	{
		auto i = writeIndex_.at(sensorId);
	} catch(std::out_of_range)
	{
		writeIndex_[sensorId] = 0;
	}
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
	return queue_.size();
}

std::string PointCloudWriter::fileNumber(int number) {
	std::ostringstream ss;
	ss << std::setw(5) << std::setfill('0') << number;
	return ss.str();
}

void PointCloudWriter::writeThreadFunction()
{
	std::cout << "Starting writeThread" << std::endl;
	while (running_) {
		if (!queue_.empty()) {
			std::lock_guard<std::mutex> lock(queue_lock_);
			if (!queue_.empty()) {
				auto id = queue_.front().sensor_id_;
				auto c = queue_.front().cloud_;
				auto i = queue_.front().image_;

				std::string cloud_name = base_filename_ + std::string("_s")
					+ std::to_string(id) + std::string("_") + fileNumber(writeIndex_[id]) + std::string(".pcd");
				std::string image_name = std::string("recorder/capture") + std::string("_s")
					+ std::to_string(id) + std::string("_") + fileNumber(writeIndex_[id]) + std::string(".jpg");

				if (c->size() > 0) {
					std::cout << "Writing: " << cloud_name << std::endl;
					pcl::io::savePCDFileBinary(cloud_name, *c.get());
				}
				if(i->getDataSize() > 0)
				{
					ofImage image;
					toOfImage(i, image);
					
					std::cout << "Writing: " << image_name << std::endl;
					image.save(image_name);
				}


				queue_.pop();
				++writeIndex_[id];
			}
			else {
				std::this_thread::sleep_for(30ms);
			}
		}
	}
}
