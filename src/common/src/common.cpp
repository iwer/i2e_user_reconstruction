#include "common/common.h"
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <ofMain.h>
#include <of-pcl-bridge/of-pcl-bridge.h>

float approxRollingAverage(float avg, float new_sample, int window)
{
	avg -= avg / window;
	avg += new_sample / window;

	return avg;
}

void downsample(recon::CloudConstPtr src, recon::CloudPtr trgt, float resolution)
{
	pcl::VoxelGrid<recon::PointType> sor;
	sor.setInputCloud(src);
	sor.setLeafSize(resolution, resolution, resolution);
	sor.filter(*trgt);
	auto indices = sor.getIndices();
}

void removeBackground(recon::CloudConstPtr src, recon::CloudPtr trgt, float min, float max)
{
	pcl::PassThrough<recon::PointType> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(min, max);
	pass.setKeepOrganized(true);
	pass.setInputCloud(src);
	pass.filter(*trgt);
}

void removeBackground(recon::CloudPtr src, recon::CloudPtr trgt, float min, float max)
{
	pcl::PassThrough<recon::PointType> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(min, max);
	pass.setKeepOrganized(true);
	pass.setInputCloud(src);
	pass.filter(*trgt);
}

void extractInOutliers(recon::CloudPtr src, pcl::PointIndices::Ptr inliers, recon::CloudPtr in_cloud, recon::CloudPtr out_cloud)
{
	pcl::ExtractIndices<recon::PointType> extract;
	extract.setInputCloud(src);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*in_cloud);

	extract.setNegative(true);
	extract.filter(*out_cloud);
}

void organizedFastMesh(recon::CloudPtr src, recon::TrianglesPtr &triangles, int edgeLength, float angleTolerance, float distanceTolerance)
{
	pcl::OrganizedFastMesh<recon::PointType> ofm;
	ofm.setTriangulationType(pcl::OrganizedFastMesh<recon::PointType>::TRIANGLE_LEFT_CUT);
	ofm.setTrianglePixelSize(edgeLength);
	ofm.setAngleTolerance(DEG2RAD(angleTolerance));
	ofm.setDistanceTolerance(distanceTolerance);
	ofm.setInputCloud(src);
	ofm.reconstruct(*triangles);
}

void drawCameraFrustum(recon::AbstractSensor::Ptr sensor)
{
	ofMatrix4x4 mat, persp;
	

	auto transl = toOfVector3(*sensor->getDepthExtrinsics()->getTranslation());
	auto rotate = toOfQuaternion(*sensor->getDepthExtrinsics()->getRotation());
	ofVec3f qaxis; float qangle;
	rotate.getRotate(qangle, qaxis);

	//mat.makeLookAtMatrix(ofVec3f(0, 0, 1), ofVec3f(0, 0, 0), ofVec3f(0, 1, 0));
	//mat.scale(1, 1, -1);
	mat.translate(transl.x, transl.y, transl.z);
	mat.rotate(qangle, qaxis.x, qaxis.y, qaxis.z);

	auto fov = sensor->getDepthIntrinsics()->getVFov();
	auto aspect = sensor->getDepthIntrinsics()->getAspectRatio();
	persp.makePerspectiveMatrix(fov, aspect, 10, 100000);
	
	mat.postMult(persp);


	ofPushMatrix();
	ofPushStyle();
	ofRotateX(180);
	ofMultMatrix(mat.getInverse());
	
	
	ofNoFill();
	ofDrawBox(0, 0, 0, 2.0f);
	ofPopMatrix();
	ofPopStyle();
}


