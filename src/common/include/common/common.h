#include <recon/PointCloudSampler.h>
#include <recon/AbstractSensor.h>
#include <ofMain.h>

float approxRollingAverage(float avg, float new_sample, int window);

void downsample(recon::CloudConstPtr cloud, recon::CloudPtr cloud_downsampled, float resolution);

void removeBackground(recon::CloudPtr src, recon::CloudPtr trgt, float min, float max);
void removeBackground(recon::CloudConstPtr src, recon::CloudPtr trgt, float min, float max);

void extractInOutliers(recon::CloudPtr src, pcl::PointIndices::Ptr inliers, recon::CloudPtr in_cloud, recon::CloudPtr out_cloud);

void organizedFastMesh(recon::CloudPtr src, recon::TrianglesPtr &triangles, int edgeLength, float angleTolerance, float distanceTolerance);
void organizedFastMesh(recon::CloudConstPtr src, recon::TrianglesPtr &triangles, int edgeLength, float angleTolerance, float distanceTolerance);


void drawCameraFrustum(recon::AbstractSensor::Ptr sensor);


ofVec2f* calculateTextureCoordinate(ofVec3f &point, ofTexture & texture, recon::AbstractSensor::Ptr sensor);
void createOfMeshWithTexCoords(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr src, boost::shared_ptr<std::vector<pcl::Vertices>> triangles, ofTexture & texture, recon::AbstractSensor::Ptr sensor, ofMesh &targetMesh);

