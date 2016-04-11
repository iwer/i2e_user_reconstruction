#pragma once

#include "ofMain.h"
#include "recon/typedefs.h"


void toOfTexture(recon::ImagePtr image, ofTexture & texture);
void createOfMeshFromPoints(recon::CloudConstPtr inputCloud, ofMesh &targetMesh);
void createOfMeshFromPoints(recon::CloudConstPtr inputCloud, ofColor color, ofMesh &targetMesh);

void toEigenVector4f(ofVec3f &ofVec, Eigen::Vector4f &pclVec);
void toEigenQuaternionf(ofQuaternion &ofQuat, Eigen::Quaternionf & pclQuat);

void toOfVector3(Eigen::Vector4f &pclVec, ofVec3f &ofVec);
void toOfQuaternion(Eigen::Quaternionf & pclQuat, ofQuaternion &ofQuat);
