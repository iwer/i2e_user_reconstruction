#include "PclOpenNI2Grabber.h"


PclOpenNI2Grabber::PclOpenNI2Grabber(void)
{
	grabber_ =  new pcl::io::OpenNI2Grabber("", pcl::io::OpenNI2Grabber::OpenNI_Default_Mode, pcl::io::OpenNI2Grabber::OpenNI_Default_Mode);

}


PclOpenNI2Grabber::~PclOpenNI2Grabber(void)
{
	delete(grabber_);
}

void PclOpenNI2Grabber::aquireFrame()
{
	// for synchronizing frame aquisition of multiple cameras
}

void PclOpenNI2Grabber::start()
{
	grabber_->start();
	// connect cloud callback to openni grabber
	boost::function<void (const CloudConstPtr&) > cloud_cb = boost::bind (&PclOpenNI2Grabber::cloud_callback, this, _1);
	cloud_connection = grabber_->registerCallback (cloud_cb);
}

void PclOpenNI2Grabber::stop()
{
	grabber_->stop();
	cloud_connection.disconnect();
}

void PclOpenNI2Grabber::cloud_callback (const CloudConstPtr& cloud)
{
	boost::mutex::scoped_lock lock (cloud_mutex_);
	cloud_ = cloud;
}

void PclOpenNI2Grabber::guiEvent(ofxUIEventArgs &e)
{

}
