#include "common/common.h"

#include <ofMain.h>
#include <of-pcl-bridge/of-pcl-bridge.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/mls.h>

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
	//auto indices = sor.getIndices();
}

void downsample(recon::CloudPtr src, recon::CloudPtr trgt, float resolution)
{
	pcl::VoxelGrid<recon::PointType> sor;
	sor.setInputCloud(src);
	sor.setLeafSize(resolution, resolution, resolution);
	sor.filter(*trgt);
	//auto indices = sor.getIndices();
}

void downsample(recon::NormalCloudPtr src, recon::NormalCloudPtr trgt, float resolution)
{
	pcl::VoxelGrid<recon::PointNormalType> sor;
	sor.setInputCloud(src);
	sor.setLeafSize(resolution, resolution, resolution);
	sor.filter(*trgt);
	//auto indices = sor.getIndices();
}

void removeBackground(recon::CloudConstPtr src, recon::CloudPtr trgt, float min, float max, bool keepOrganized)
{
	pcl::PassThrough<recon::PointType> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(min, max);
	pass.setKeepOrganized(keepOrganized);
	pass.setInputCloud(src);
	pass.filter(*trgt);
}

void removeBackground(recon::CloudPtr src, recon::CloudPtr trgt, float min, float max, bool keepOrganized)
{
	pcl::PassThrough<recon::PointType> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(min, max);
	pass.setKeepOrganized(keepOrganized);
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


void organizedFastMesh(recon::CloudPtr src, recon::TrianglesPtr& triangles, int edgeLength, float angleTolerance, float distanceTolerance)
{
	pcl::OrganizedFastMesh<recon::PointType> ofm;
	ofm.setTriangulationType(pcl::OrganizedFastMesh<recon::PointType>::TRIANGLE_LEFT_CUT);
	ofm.setTrianglePixelSize(edgeLength);
	ofm.setAngleTolerance(DEG2RAD(angleTolerance));
	ofm.setDistanceTolerance(distanceTolerance);
	ofm.setInputCloud(src);
	ofm.reconstruct(*triangles);
}


void organizedFastMesh(recon::CloudConstPtr src, recon::TrianglesPtr& triangles, int edgeLength, float angleTolerance, float distanceTolerance)
{
	pcl::OrganizedFastMesh<recon::PointType> ofm;
	ofm.setTriangulationType(pcl::OrganizedFastMesh<recon::PointType>::TRIANGLE_LEFT_CUT);
	ofm.setTrianglePixelSize(edgeLength);
	ofm.setAngleTolerance(DEG2RAD(angleTolerance));
	ofm.setDistanceTolerance(distanceTolerance);
	ofm.setInputCloud(src);
	ofm.reconstruct(*triangles);
}

void movingLeastSquaresSmoothing(recon::NormalCloudPtr src, recon::NormalCloudPtr trgt, float smoothRadius, float resolution)
{
	// Create a KD-Tree
	pcl::search::KdTree<recon::PointType>::Ptr tree(new pcl::search::KdTree<recon::PointType>);

	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<recon::PointType, recon::PointNormalType> mls;
	
	recon::CloudPtr normalRemovedCloud(new recon::Cloud);
	pcl::copyPointCloud<pcl::PointXYZRGBNormal, recon::PointType>(*src, *normalRemovedCloud);
	std::cout << "WO Normals: " << normalRemovedCloud->size() << std::endl;
	mls.setComputeNormals(true);

	// Set parameters
	mls.setPolynomialFit(true);
	mls.setPolynomialOrder(2);
	// TODO: check other upsampling methods

	// seems to work best for me
	mls.setDilationVoxelSize(resolution);
	mls.setUpsamplingMethod(pcl::MovingLeastSquares<recon::PointType, recon::PointNormalType>::VOXEL_GRID_DILATION);

	// still quite noisy	
	//mls.setPointDensity(10);
	//mls.setUpsamplingMethod(pcl::MovingLeastSquares<recon::PointType, recon::PointNormalType>::RANDOM_UNIFORM_DENSITY);

	// pretty noisy
	//mls.setDistinctCloud(normalRemovedCloud);
	//mls.setUpsamplingMethod(pcl::MovingLeastSquares<recon::PointType, recon::PointNormalType>::DISTINCT_CLOUD);

	// does not work at all? (maybe parameter related)
	//mls.setUpsamplingStepSize(resolution);
	//mls.setUpsamplingRadius(resolution * 3);
	//mls.setUpsamplingMethod(pcl::MovingLeastSquares<recon::PointType, recon::PointNormalType>::SAMPLE_LOCAL_PLANE);

	mls.setSearchMethod(tree);
	mls.setSearchRadius(smoothRadius);

	// Reconstruct
	mls.setInputCloud(normalRemovedCloud);
	mls.setDistinctCloud(normalRemovedCloud);
	mls.process(*trgt);

}

void calculatePointNormals(recon::CloudPtr src, recon::NormalCloudPtr trgt, int normalKNeighbours)
{
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n_;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals_(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr justPoints(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud<recon::PointType, pcl::PointXYZ>(*src, *justPoints);

	// Normal estimation
	tree_->setInputCloud(justPoints);
	n_.setNumberOfThreads(8);
	n_.setInputCloud(justPoints);
	n_.setSearchMethod(tree_);
	n_.setKSearch(normalKNeighbours);
	n_.compute(*normals_);

	// Concatenate the XYZ and normal fields
	pcl::concatenateFields(*src.get(), *normals_.get(), *trgt.get());
}

void greedyProjectionMesh(recon::NormalCloudPtr src, recon::TrianglesPtr& triangles, float maxEdgeLength, float mu, int maxNearestNeighbours,
                          float maxSurfaceDegree, float minTriDegree, float maxTriDegree)
{
	if (src->size() > 0) {

		pcl::GreedyProjectionTriangulation<recon::PointNormalType> gp3_;


		// Create search tree
		pcl::search::KdTree<recon::PointNormalType>::Ptr tree2(new pcl::search::KdTree<recon::PointNormalType>);
		tree2->setInputCloud(src);

		// Set parameters
		gp3_.setSearchRadius(maxEdgeLength);
		gp3_.setMu(mu);
		gp3_.setMaximumNearestNeighbors(maxNearestNeighbours);
		gp3_.setMaximumSurfaceAngle(DEG2RAD(maxSurfaceDegree));
		gp3_.setMinimumAngle(DEG2RAD(minTriDegree));
		gp3_.setMaximumAngle(DEG2RAD(maxTriDegree));
		gp3_.setNormalConsistency(true);
		gp3_.setConsistentVertexOrdering(true);

		// Get result
		gp3_.setInputCloud(src);
		gp3_.setSearchMethod(tree2);
		gp3_.reconstruct(*triangles);

		// Additional vertex information
		//auto parts = gp3_.getPartIDs();
		//auto states = gp3_.getPointStates();
	}
}

void drawCameraFrustum(recon::AbstractSensor::Ptr sensor)
{
	ofMatrix4x4 mat, persp;

	auto transl = toOfVector3(*sensor->getDepthExtrinsics()->getTranslation());
	auto rotate = toOfQuaternion(*sensor->getDepthExtrinsics()->getRotation());


	ofVec3f qaxis;
	float qangle;
	rotate.getRotate(qangle, qaxis);
	mat.translate(-transl.x, -transl.y, -transl.z);
	mat.rotate(180, 1, 0, 0);
	mat.rotate(qangle, -qaxis.x, qaxis.y, qaxis.z);

	auto fov = sensor->getDepthIntrinsics()->getVFov();
	auto aspect = sensor->getDepthIntrinsics()->getAspectRatio();
	persp.makePerspectiveMatrix(fov, aspect, 10, 1000);

	mat.postMult(persp);

	ofPushMatrix();
	ofPushStyle();
	ofMultMatrix(mat.getInverse());
	ofNoFill();
	ofSetColor(getSensorColor(sensor->getId()));
	ofDrawBox(0, 0, 0, 2.0f);
	//ofDrawAxis(20);
	ofPopMatrix();
	ofPopStyle();
}


ofVec2f calculateTextureCoordinate(ofVec3f& point, float texwidth, float texheight, recon::AbstractSensor::Ptr sensor, bool inCameraSpace)
{
	ofMatrix4x4 mat, persp;
	auto intrinsics = sensor->getDepthIntrinsics();


	auto transl = toOfVector3(*sensor->getDepthExtrinsics()->getTranslation());//
	auto rotate = toOfQuaternion(*sensor->getDepthExtrinsics()->getRotation());//

	ofVec3f qaxis;//
	float qangle;//

	if (!inCameraSpace)
	{
		rotate.getRotate(qangle, qaxis);//
		mat.translate(-transl.x, -transl.y, -transl.z);//
	}

	// counter pcl s positive z-direction
	mat.rotate(180, 1, 0, 0);

	if (!inCameraSpace)
	{
		mat.rotate(qangle, -qaxis.x, qaxis.y, qaxis.z);//
	}
	// create ViewProjection Matrix
	persp.makePerspectiveMatrix(intrinsics->getVFov(), intrinsics->getAspectRatio(), 10, 100000);
	mat.postMult(persp);

	// project 3d point to 2d camera plane to obtain texcoords
	ofVec4f projectedPoint = ofVec4f(point.x, point.y, point.z, 1) * mat;
	// scale to -1, 1
	projectedPoint.x /= projectedPoint.w;
	projectedPoint.y /= projectedPoint.w;
	// scale to 0, width
	auto new_x = (projectedPoint.x + 1) * .5 * texwidth;
	auto new_y = (projectedPoint.y + 1) * .5 * texheight;
	return ofVec2f(new_x, new_y);
}


void createOfMeshWithTexCoords(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr src,
                               boost::shared_ptr<std::vector<pcl::Vertices>> triangles,
                               ofTexture& texture,
                               recon::AbstractSensor::Ptr sensor,
                               ofMesh& targetMesh)
{
	if (triangles && src)
	{
		// triangle inputMesh
		targetMesh.clear();
		targetMesh.setMode(OF_PRIMITIVE_TRIANGLES);
		pcl::PointXYZRGB p;

		for (auto& t : *triangles)
		{
			for (auto& pointindex : t.vertices)
			{
				p = src->at(pointindex);
				auto ofp = ofVec3f(p.x * 1000, p.y * 1000, p.z * 1000);
				targetMesh.addVertex(ofp);
				targetMesh.addColor(ofColor(p.r, p.g, p.b));
				targetMesh.addTexCoord(calculateTextureCoordinate(ofp, texture.getWidth(), texture.getHeight(), sensor, true));
			}
		}
	}
}

void createOfMeshWithCombinedTexCoords(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr src,
                                       boost::shared_ptr<std::vector<pcl::Vertices>> triangles,
                                       ofTexture& texture,
                                       ofRectangle& texturelayout,
                                       recon::AbstractSensor::Ptr sensor,
                                       ofMesh& targetMesh)
{
	if (triangles && src)
	{
		// triangle inputMesh
		targetMesh.clear();
		targetMesh.setMode(OF_PRIMITIVE_TRIANGLES);
		pcl::PointXYZRGB p;

		for (auto& t : *triangles)
		{
			for (auto& pointindex : t.vertices)
			{
				p = src->at(pointindex);
				auto ofp = ofVec3f(p.x * 1000, p.y * 1000, p.z * 1000);
				targetMesh.addVertex(ofp);
				targetMesh.addColor(ofColor(p.r, p.g, p.b));
				//calculate texcoord on single texture
				auto localTexcoord = calculateTextureCoordinate(ofp, texturelayout.getWidth(), texturelayout.getHeight(), sensor, false);
				//std::cout << localTexcoord << "\t->\t";
				//shift according to texture layout
				localTexcoord += texturelayout.getTopLeft();
				//std::cout << localTexcoord << std::endl;
				targetMesh.addTexCoord(localTexcoord);
			}
		}
	}
}

void createOfMeshWithCombinedTexCoords(pcl::PointCloud<pcl::PointXYZRGBNormal>::ConstPtr src,
                                       boost::shared_ptr<std::vector<pcl::Vertices>> triangles,
                                       std::vector<ofVec2f>& texcoords,
                                       ofMesh& targetMesh)
{
	if (triangles && src)
	{
		// triangle inputMesh
		targetMesh.clear();
		targetMesh.setMode(OF_PRIMITIVE_TRIANGLES);
		pcl::PointXYZRGBNormal p;

		for (auto& t : *triangles)
		{
			for (auto& pointindex : t.vertices)
			{
				p = src->at(pointindex);
				auto ofp = ofVec3f(p.x * 1000, p.y * 1000, p.z * 1000);
				targetMesh.addVertex(ofp);
				targetMesh.addColor(ofColor(p.r, p.g, p.b));
				targetMesh.addNormal(ofVec3f(p.normal_x, p.normal_y, p.normal_z));
				targetMesh.addTexCoord(texcoords.at(pointindex));
			}
		}
	}
}

void createOfMeshFromPclTextureMesh(pcl::TextureMeshPtr mesh,
	recon::NormalCloudPtr cloud,
	std::vector<ofRectangle>& texturelayout,
	std::map<int, recon::AbstractSensor::Ptr> sensors,
	std::map<int, std::shared_ptr<ofImage>> &texture,
	ofMesh &targetMesh,
	bool camIndexColor)
{
	targetMesh.clear();
	targetMesh.setMode(OF_PRIMITIVE_TRIANGLES);

	//pcl::PointCloud<pcl::PointXYZ> local_cloud;
	//pcl::fromPCLPointCloud2(mesh->cloud, local_cloud);

	unsigned sub_mesh_idx = 0;
	for (auto &sub_mesh : mesh->tex_polygons)
	{
		unsigned tri_mesh_idx = 0;

		for (auto &t : sub_mesh) {
			
			for (auto &pointindex : t.vertices)
			{
				auto p = cloud->at(pointindex);
				auto ofp = ofVec3f(p.x * 1000, p.y * 1000, p.z * 1000);
				auto normal = ofVec3f(p.normal_x, p.normal_y, p.normal_z);
				targetMesh.addVertex(ofp);
				targetMesh.addNormal(normal);
				if(sub_mesh_idx < sensors.size()){
					auto sensor = sensors[sub_mesh_idx];
					auto local_texcoord = calculateTextureCoordinate(ofp, texturelayout[sub_mesh_idx].getWidth(), texturelayout[sub_mesh_idx].getHeight(), sensor, false);

					if (camIndexColor) {
						targetMesh.addColor(getSensorColor(sub_mesh_idx));
					} else
					{
						targetMesh.addColor(ofColor(p.r, p.g, p.b));
					}
					local_texcoord += texturelayout[sub_mesh_idx].getTopLeft();
					targetMesh.addTexCoord(local_texcoord);
				}
			}
			++tri_mesh_idx;
		}
		++sub_mesh_idx;
	}
}

ofColor getSensorColor(int sensorId)
{
	ofColor c;
	switch (sensorId)
	{
	case 0:
		c = ofColor(255, 0, 0);
		break;
	case 1:
		c = ofColor(0, 255, 0);
		break;
	case 2:
		c = ofColor(0, 0, 255);
		break;
	case 3:
		c = ofColor(255, 255, 0);
		break;
	default:
		c = ofColor(255, 255, 255);
		break;
	}
	return c;
}
