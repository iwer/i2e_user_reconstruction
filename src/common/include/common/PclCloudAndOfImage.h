#include <recon/typedefs.h>
#include <ofImage.h>
#include "PclCloudAndImage.h"

class PclCloudAndOfImage
{
public:
	typedef std::shared_ptr<PclCloudAndOfImage> Ptr;
	PclCloudAndOfImage(int id, recon::CloudPtr cloud, std::shared_ptr<ofImage> image)
		: sensor_id_(id)
		, cloud_(cloud)
		, image_(image)
	{}
	int sensor_id_;
	recon::CloudPtr cloud_;
	std::shared_ptr<ofImage> image_;
};
