#include <recon/typedefs.h>
#include <ofImage.h>

class OfMeshAndPixels
{
public:
	typedef std::shared_ptr<OfMeshAndPixels> Ptr;
	OfMeshAndPixels(std::shared_ptr<ofMesh> mesh, std::shared_ptr<ofPixels> image)
		: mesh_(mesh)
		, image_(image)
	{}
	std::shared_ptr<ofMesh> mesh_;
	std::shared_ptr<ofPixels> &image_;
};
