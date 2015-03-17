#pragma once
#include "typedefs.h"
#include <boost/thread.hpp>

class AbstractMeshProcessor
{
public:
	AbstractMeshProcessor(void);
	virtual ~AbstractMeshProcessor(void);

	void setInputCloud(CloudConstPtr);
	CloudConstPtr getInputCloud();
	TrianglesPtr getTriangles();

	virtual void processData() = 0;
protected:
	CloudConstPtr inputCloud_;
	TrianglesPtr triangles_;

	boost::mutex cloud_mutex_;
	boost::mutex triangle_mutex_;
};

