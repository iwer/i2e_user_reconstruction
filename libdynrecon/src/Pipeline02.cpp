#include "Pipeline02.h"


Pipeline02::Pipeline02(boost::signals2::signal<void (float)> * minDepUpdate, 
					   boost::signals2::signal<void (float)> * maxDepUpdate, 
					   boost::signals2::signal<void (float)> * triangleSizeUpdate)
{


	pp_ = &d;
	mp_ = &g;

	minDepUpdate->connect(boost::bind(&DepthThreshold::setDepthThresholdMin, &d, _1));
	maxDepUpdate->connect(boost::bind(&DepthThreshold::setDepthThresholdMax, &d, _1));
	triangleSizeUpdate->connect(boost::bind(&PointCloudSampler::setResolution, &s, _1));
}


Pipeline02::~Pipeline02(void)
{
}

void Pipeline02::processData()
{
	pp_->setInputCloud(cloud_);
	pp_->processData();
	s.setInputCloud(pp_->getOutputCloud());
	s.processData();
	mp_->setInputCloud(s.getOutputCloud());
	mp_->processData();
	meshCloud_ = mp_->getInputCloud();
	triangles_ = mp_->getTriangles();
}