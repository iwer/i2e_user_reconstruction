#include "PointCloudWriter.h"
#include <chrono>
#include <pcl/io/file_io.h>
#include <ofTexture.h>
#include <of-pcl-bridge/of-pcl-bridge.h>

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


void PointCloudWriter::enquePointcloudForWriting(int sensorId, recon::CloudConstPtr cloud, recon::ImagePtr image)
{
	std::lock_guard<std::mutex> lock(queue_lock_);
	queue_.push(SaveTriplet(sensorId, cloud, image));
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

std::string PointCloudWriter::fileNumber() {
	std::ostringstream ss;
	ss << std::setw(5) << std::setfill('0') << writeIndex_;
	return ss.str();
}

void PointCloudWriter::writeThreadFunction()
{
	std::cout << "Starting writeThread" << std::endl;
	while (running_) {
		if (!queue_.empty()) {
			std::lock_guard<std::mutex> lock(queue_lock_);
			if (!queue_.empty()) {
				auto id = queue_.front().id_;
				auto c = queue_.front().cloud_;
				auto i = queue_.front().image_;

				std::string cloud_name = base_filename_ + std::string("_s")
					+ std::to_string(id) + std::string("_") + fileNumber() + std::string(".pcd");
				std::cout << "Writing: " << cloud_name << std::endl;
				std::string image_name = base_filename_ + std::string("_s")
					+ std::to_string(id) + std::string("_") + fileNumber() + std::string(".png");

				if (c->size() > 0) {
					pcl::io::savePCDFileBinary(cloud_name, *c.get());
				}
				if(i->getDataSize() > 0)
				{
					ofImage t;
					toOfTexture(i, t.getTextureReference());

					t.save(image_name);
				}


				queue_.pop();
				++writeIndex_;
			}
			else {
				std::this_thread::sleep_for(30ms);
			}
		}
	}
}
