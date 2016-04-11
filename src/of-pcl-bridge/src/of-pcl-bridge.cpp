#include "of-pcl-bridge/of-pcl-bridge.h"

void toOfTexture(recon::ImagePtr image, ofTexture & texture)
{
	auto width = image->getWidth();
	auto height = image->getHeight();
	auto encoding = image->getEncoding();

	if (encoding == pcl::io::Image::Encoding::RGB)
	{
		auto data = static_cast<const unsigned char *>(image->getData());
		texture.loadData(data, width, height, GL_RGB);
	}
}

void createOfMeshFromPoints(recon::CloudConstPtr inputCloud, ofMesh &targetMesh)
{
	targetMesh.clear();
	targetMesh.setMode(OF_PRIMITIVE_POINTS);
	for (auto &p : inputCloud->points) {
		targetMesh.addVertex(ofVec3f(p.x * 1000, p.y * 1000, p.z * 1000));
		targetMesh.addColor(ofColor(p.r, p.g, p.b));
	}
}

void createOfMeshFromPoints(recon::CloudConstPtr inputCloud, ofColor color, ofMesh &targetMesh)
{
	if (inputCloud) {
		// triangle inputMesh
		targetMesh.clear();
		targetMesh.setMode(OF_PRIMITIVE_POINTS);
		for (auto &p : inputCloud->points) {
			targetMesh.addVertex(ofVec3f(p.x * 1000, p.y * 1000, p.z * 1000));
			targetMesh.addColor(color);
		}
	}
}

void toEigenVector4f(ofVec3f &ofVec, Eigen::Vector4f &pclVec)
{
	pclVec.x() = ofVec.x;
	pclVec.y() = ofVec.y;
	pclVec.z() = ofVec.z;
	pclVec.w() = 0;
}
void toEigenQuaternionf(ofQuaternion &ofQuat, Eigen::Quaternionf & pclQuat)
{
	pclQuat.w() = ofQuat.w();
	pclQuat.x() = ofQuat.x();
	pclQuat.y() = ofQuat.y();
	pclQuat.z() = ofQuat.z();
}

void toOfVector3(Eigen::Vector4f &pclVec, ofVec3f &ofVec)
{
	ofVec.set(pclVec.x(), pclVec.y(), pclVec.z());
}
void toOfQuaternion(Eigen::Quaternionf & pclQuat, ofQuaternion &ofQuat)
{
	ofQuat.set(pclQuat.x(), pclQuat.y(), pclQuat.z(), pclQuat.w());
}