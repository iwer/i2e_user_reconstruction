//
// Created by elresidente on 09.10.17.
//

#ifndef I2E_USER_RECONSTRUCTION_NETKINECTSENSOR_H
#define I2E_USER_RECONSTRUCTION_NETKINECTSENSOR_H


#include <Client.h>
#include <recon/AbstractSensor.h>
#include "NetKinectPointCloudGenerator.h"

class NetKinectSensor : public recon::AbstractSensor {
public:
    const static unsigned KINECT_WIDTH = 640;
    const static unsigned KINECT_HEIGHT = 480;

    NetKinectSensor(Client * client, int id);
    void setBackGroundImpl(void);

private:
    NetKinectPointCloudGenerator * netkinect_cloud_source_;
};


#endif //I2E_USER_RECONSTRUCTION_NETKINECTSENSOR_H
