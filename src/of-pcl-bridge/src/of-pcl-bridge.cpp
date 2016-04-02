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
		//targetMesh.addColor(cloudColors[meshIndex]);
	}
}