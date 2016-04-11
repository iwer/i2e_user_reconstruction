#include "of-pcl-bridge/of-pcl-bridge.h"

void toOfTexture(boost::shared_ptr<pcl::io::Image> image, ofTexture & texture)
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

void createOfMeshFromPoints(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud, ofMesh &targetMesh)
{
	targetMesh.clear();
	targetMesh.setMode(OF_PRIMITIVE_POINTS);
	for (auto &p : inputCloud->points) {
		targetMesh.addVertex(ofVec3f(p.x * 1000, p.y * 1000, p.z * 1000));
		targetMesh.addColor(ofColor(p.r, p.g, p.b));
	}
}

void createOfMeshFromPoints(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr inputCloud, ofColor color, ofMesh &targetMesh)
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

Eigen::Vector4f toEigenVector4f(ofVec3f& ofVec)
{
	Eigen::Vector4f ret;
	toEigenVector4f(ofVec, ret);
	return ret;
}

Eigen::Quaternionf toEigenQuaternionf(ofQuaternion& ofQuat)
{
	Eigen::Quaternionf ret;
	toEigenQuaternionf(ofQuat, ret);
	return ret;
}

ofVec3f toOfVector3(Eigen::Vector4f& pclVec)
{
	ofVec3f ret;
	toOfVector3(pclVec, ret);
	return ret;
}

ofQuaternion toOfQuaternion(Eigen::Quaternionf& pclQuat)
{
	ofQuaternion ret;
	toOfQuaternion(pclQuat, ret);
	return  ret;
}

void toEigenVector4f(ofVec3f &ofVec, Eigen::Vector4f &pclVec)
{
	pclVec.x() = ofVec.x / 1000;
	pclVec.y() = ofVec.y / 1000;
	pclVec.z() = ofVec.z / 1000;
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
	ofVec.set(pclVec.x() * 1000, pclVec.y() * 1000, pclVec.z() * 1000);
}
void toOfQuaternion(Eigen::Quaternionf & pclQuat, ofQuaternion &ofQuat)
{
	ofQuat.set(pclQuat.x(), pclQuat.y(), pclQuat.z(), pclQuat.w());
}