#include <recon/PointCloudSampler.h>
#include <recon/AbstractSensor.h>

float approxRollingAverage(float avg, float new_sample, int window);
void downsample(recon::CloudConstPtr cloud, recon::CloudPtr cloud_downsampled, float resolution);
void removeBackground(recon::CloudPtr src, recon::CloudPtr trgt, float min, float max);
void removeBackground(recon::CloudConstPtr src, recon::CloudPtr trgt, float min, float max);
void extractInOutliers(recon::CloudPtr src, pcl::PointIndices::Ptr inliers, recon::CloudPtr in_cloud, recon::CloudPtr out_cloud);
void organizedFastMesh(recon::CloudPtr src, recon::TrianglesPtr &triangles, int edgeLength, float angleTolerance, float distanceTolerance);


void drawCameraFrustum(recon::AbstractSensor::Ptr sensor);

