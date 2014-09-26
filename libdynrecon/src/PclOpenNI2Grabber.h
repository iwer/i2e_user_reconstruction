#pragma once

#include "typedefs.h"
#include "AbstractPointCloudGenerator.h"
#include <pcl/io/openni2_grabber.h>

class PclOpenNI2Grabber :
	public AbstractPointCloudGenerator
{
public:
	PclOpenNI2Grabber(void);
	~PclOpenNI2Grabber(void);

	void aquireFrame();
	void cloud_callback (const CloudConstPtr& cloud);

	void start();
	void stop();

private:
	pcl::io::OpenNI2Grabber * grabber_;
	boost::signals2::connection cloud_connection;

};

