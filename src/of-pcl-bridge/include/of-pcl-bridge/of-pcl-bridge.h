#pragma once

#include "ofMain.h"
#include "pcl/common/common.h"
#include <pcl/io/image.h>


void toOfTexture(boost::shared_ptr<pcl::io::Image> image, ofTexture & texture);
void createOfMeshFromPoints(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud, ofMesh &targetMesh);
void createOfMeshFromPoints(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud, ofColor color, ofMesh &targetMesh);

void toEigenVector4f(ofVec3f &ofVec, Eigen::Vector4f &pclVec);
Eigen::Vector4f toEigenVector4f(ofVec3f &ofVec);
void toEigenQuaternionf(ofQuaternion &ofQuat, Eigen::Quaternionf & pclQuat);
Eigen::Quaternionf toEigenQuaternionf(ofQuaternion &ofQuat);

void toOfVector3(Eigen::Vector4f &pclVec, ofVec3f &ofVec);
ofVec3f toOfVector3(Eigen::Vector4f &pclVec);
void toOfQuaternion(Eigen::Quaternionf & pclQuat, ofQuaternion &ofQuat);
ofQuaternion toOfQuaternion(Eigen::Quaternionf & pclQuat);
