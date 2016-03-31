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