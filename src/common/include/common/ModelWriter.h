#pragma once
#include <string>
#include <ofImage.h>
#include "OfMeshAndPixels.h"

class ModelWriter
{
public:
	ModelWriter();
	~ModelWriter();

	void setBaseFileName(std::string & filename);
	void enqueModelForWriting(std::shared_ptr<ofMesh> mesh, std::shared_ptr<ofPixels> image);

	void start();
	void stop();

	int getQueueLength();
	void saveFrame(std::shared_ptr<ofMesh> mesh, std::shared_ptr<ofPixels> image);
private:
	bool running_;

	int writeIndex_;
	std::string base_filename_;

	std::queue<OfMeshAndPixels::Ptr> queue_;

	std::mutex queue_lock_;

	std::thread * write_thread_;

	void writeThreadFunction();
	std::string fileNumber(int number);
};

