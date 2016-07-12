#include "common/ModelWriter.h"



ModelWriter::ModelWriter()
	: running_(false)
{
}


ModelWriter::~ModelWriter()
{
	if (running_) {
		stop();
	}
}

void ModelWriter::setBaseFileName(std::string& filename)
{
	base_filename_ = filename;
}

void ModelWriter::enqueModelForWriting(std::shared_ptr<ofMesh> mesh, std::shared_ptr<ofPixels> image)
{
	std::lock_guard<std::mutex> lock(queue_lock_);
	OfMeshAndPixels::Ptr model(new OfMeshAndPixels(mesh, image));
	queue_.push(model);
}

void ModelWriter::start()
{
	running_ = true;
	write_thread_ = new std::thread(&ModelWriter::writeThreadFunction, this);
}

void ModelWriter::stop()
{
	running_ = false;
	write_thread_->join();
}

int ModelWriter::getQueueLength()
{
	std::lock_guard<std::mutex> lock(queue_lock_);
	return queue_.size();
}

void ModelWriter::saveFrame(std::shared_ptr<ofMesh> mesh, std::shared_ptr<ofPixels> image)
{
	auto mesh_name = std::string("reconstructed/frame_") + fileNumber(writeIndex_) + std::string(".ply");
	auto image_name = std::string("reconstructed/frame_") + fileNumber(writeIndex_) + std::string(".png");

	mesh->save(mesh_name);
	ofSaveImage(*image, image_name);
	++writeIndex_;

}

void ModelWriter::writeThreadFunction()
{
	std::cout << "Starting writeThread" << std::endl;
	while (running_) {
		if (!queue_.empty()) {
			std::lock_guard<std::mutex> lock(queue_lock_);
			if (!queue_.empty()) {
				auto mesh = queue_.front()->mesh_;
				auto image = queue_.front()->image_;

				saveFrame(mesh, image);


				queue_.pop();
			}
			else {
				std::this_thread::sleep_for(30ms);
			}
		}
	}
}

std::string ModelWriter::fileNumber(int number)
{
	std::ostringstream ss;
	ss << std::setw(5) << std::setfill('0') << number;
	return ss.str();
}
