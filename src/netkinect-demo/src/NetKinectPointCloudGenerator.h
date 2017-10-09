//
// Created by elresidente on 09.10.17.
//

#ifndef I2E_USER_RECONSTRUCTION_NETKINECTPOINTCLOUDGENERATOR_H
#define I2E_USER_RECONSTRUCTION_NETKINECTPOINTCLOUDGENERATOR_H

#include <recon/typedefs.h>
#include <recon/AbstractPointCloudGenerator.h>
#include <Client.h>

class NetKinectPointCloudGenerator : public recon::AbstractPointCloudGenerator{
public:
    NetKinectPointCloudGenerator(unsigned width, unsigned height);
    void start();
    void stop();
    void aquireFrame();

    recon::CloudConstPtr getOutputCloud();
    recon::ImagePtr getOutputImage();
    void set_netkinect_client(Client * netkinect_client);

    // extend pcl's FrameWrapper to fill an pcl::io::Image
    class ImageMetadata : public pcl::io::FrameWrapper {
    public:
        ImageMetadata(void * data, unsigned dataSize, unsigned width, unsigned height, unsigned frameID, pcl::uint64_t timestamp);
        const void*	getData() const;
        unsigned getDataSize() const;
        unsigned getWidth() const;
        unsigned getHeight() const;
        unsigned getFrameID() const;

        // Microseconds from some arbitrary start point
        pcl::uint64_t getTimestamp() const;
    private:
        void * data_;
        unsigned dataSize_;
        unsigned width_;
        unsigned height_;
        unsigned frameID_;
        pcl::uint64_t timestamp_;
    };

private:
    Client * netkinect_client_;
    unsigned width_;
    unsigned height_;
};


#endif //I2E_USER_RECONSTRUCTION_NETKINECTPOINTCLOUDGENERATOR_H
