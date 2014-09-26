#include "AbstractPointCloudGenerator.h"


AbstractPointCloudGenerator::AbstractPointCloudGenerator(void) :
	cloud_(new Cloud),
	temp_cloud_(new Cloud)
{
}


AbstractPointCloudGenerator::~AbstractPointCloudGenerator(void)
{
}

CloudConstPtr AbstractPointCloudGenerator::getOutputCloud()
{
	if(cloud_mutex_.try_lock()) {
		temp_cloud_.swap(cloud_);
	}
	return temp_cloud_;
}

