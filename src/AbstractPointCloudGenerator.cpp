#include "AbstractPointCloudGenerator.h"


AbstractPointCloudGenerator::AbstractPointCloudGenerator(void)
{
}


AbstractPointCloudGenerator::~AbstractPointCloudGenerator(void)
{
}

CloudPtr AbstractPointCloudGenerator::getOutputCloud()
{
	return cloud_;
}

