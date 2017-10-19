//
// Created by elresidente on 09.10.17.
//

#include "NetKinectPointCloudGenerator.h"
#include <pcl/io/image_rgb24.h>

NetKinectPointCloudGenerator::NetKinectPointCloudGenerator(unsigned width, unsigned height)
        : AbstractPointCloudGenerator()
{
}

void NetKinectPointCloudGenerator::start()
{
}

void NetKinectPointCloudGenerator::stop()
{

}

void NetKinectPointCloudGenerator::aquireFrame()
{
}

recon::CloudConstPtr NetKinectPointCloudGenerator::getOutputCloud() {
    int size = 0;
    float** cloud = nullptr;
    //if(netkinect_client_->isAbleToDeliverData()) {
	    size = netkinect_client_->getCloud(cloud, size);

	    //Encode cloud into pcl::PointCloud<pcl::PointXYZ>
	    recon::CloudPtr newcloud;
	    for (int i = 0; i <= size - 3; i += 3) {
		recon::PointType p;
		p.x = *cloud[i];
		p.y = *cloud[i + 1];
		p.z = *cloud[i + 2];

		newcloud->push_back(p);
	    }
	    // save for collection
	    cloud_mutex_.lock();
	    cloud_ = newcloud;
	    cloud_mutex_.unlock();
    //}

    return recon::AbstractPointCloudGenerator::getOutputCloud();
}

recon::ImagePtr NetKinectPointCloudGenerator::getOutputImage() {
    int size = 0;
    char** video = nullptr;

    //if(netkinect_client_->isAbleToDeliverData()) {
	    netkinect_client_->getVideo(video, size);
	    // Encode fvideo_data into pcl::io::Image
	    boost::shared_ptr<NetKinectPointCloudGenerator::ImageMetadata> metadata = boost::shared_ptr<NetKinectPointCloudGenerator::ImageMetadata>(
		    new NetKinectPointCloudGenerator::ImageMetadata((void*)video, size, width_, height_, 1, 0));
	    image_mutex_.lock();
	    image_ = recon::ImagePtr(new pcl::io::ImageRGB24(metadata));
	    image_mutex_.unlock();
    //}
    return recon::AbstractPointCloudGenerator::getOutputImage();
}

void NetKinectPointCloudGenerator::set_netkinect_client(Client * netkinect_client) {
    netkinect_client_ = netkinect_client;
}

NetKinectPointCloudGenerator::ImageMetadata::ImageMetadata(void * data, unsigned dataSize, unsigned width, unsigned height, unsigned frameID, pcl::uint64_t timestamp)
        : data_(data)
        , dataSize_(dataSize)
        , width_(width)
        , height_(height)
        , frameID_(frameID)
        , timestamp_(timestamp)
{
}

const void * NetKinectPointCloudGenerator::ImageMetadata::getData() const
{
    return data_;
}

unsigned NetKinectPointCloudGenerator::ImageMetadata::getDataSize() const
{
    return dataSize_;
}

unsigned NetKinectPointCloudGenerator::ImageMetadata::getWidth() const
{
    return width_;
}

unsigned NetKinectPointCloudGenerator::ImageMetadata::getHeight() const
{
    return height_;
}

unsigned NetKinectPointCloudGenerator::ImageMetadata::getFrameID() const
{
    return frameID_;
}

pcl::uint64_t NetKinectPointCloudGenerator::ImageMetadata::getTimestamp() const
{
    return timestamp_;
}
