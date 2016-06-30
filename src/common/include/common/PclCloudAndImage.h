#include <recon/typedefs.h>

class PclCloudAndImage
{
public:
	typedef std::shared_ptr<PclCloudAndImage> Ptr;
	PclCloudAndImage(int id, recon::CloudConstPtr cloud, recon::ImagePtr image)
		: sensor_id_(id)
		, cloud_(cloud)
		, image_(image)
	{}
	int sensor_id_;
	recon::CloudConstPtr cloud_;
	recon::ImagePtr image_;
};