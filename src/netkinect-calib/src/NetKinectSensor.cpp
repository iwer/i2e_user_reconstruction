//
// Created by elresidente on 09.10.17.
//

#include "NetKinectSensor.h"

NetKinectSensor::NetKinectSensor(Client * client, int id)
        : AbstractSensor()
        , netkinect_cloud_source_(new NetKinectPointCloudGenerator(KINECT_WIDTH, KINECT_HEIGHT))
{
    cloudSource_= netkinect_cloud_source_;
    netkinect_cloud_source_->set_netkinect_client(client);
    sensorId_ = id;
}

void NetKinectSensor::setBackGroundImpl(void) {

}


