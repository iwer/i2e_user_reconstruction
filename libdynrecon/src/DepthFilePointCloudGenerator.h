#include "AbstractPointCloudGenerator.h"

class DepthFilePointCloudGenerator : AbstractPointCloudGenerator {


public:
	DepthFilePointCloudGenerator();
	~DepthFilePointCloudGenerator();

	void aquireFrame() override;

	void start() override;

	void stop() override;


	void loadDepthImageFromFile(std::string fileName);
};
