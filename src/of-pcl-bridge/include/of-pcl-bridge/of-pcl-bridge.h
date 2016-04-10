#pragma once

#include "ofMain.h"
#include "recon/typedefs.h"


void toOfTexture(recon::ImagePtr image, ofTexture & texture);
void createOfMeshFromPoints(recon::CloudConstPtr inputCloud, ofMesh &targetMesh);
void createOfMeshFromPoints(recon::CloudConstPtr inputCloud, ofColor color, ofMesh &targetMesh);
