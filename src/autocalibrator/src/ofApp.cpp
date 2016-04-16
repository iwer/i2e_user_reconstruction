#include "ofApp.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>

//--------------------------------------------------------------
void ofApp::setup()
{
	std::chrono::seconds five_sec(3);
	static_time_to_snapshot_ = five_sec;
	cloudColors[0].set(255, 0, 0);
	cloudColors[1].set(0, 255, 0);
	cloudColors[2].set(0, 0, 255);
	cloudColors[3].set(255, 255, 0);

	// sensor setup
	recon::SensorFactory sensorFac;

	auto nSensors = sensorFac.checkConnectedDevices(true);
	for (int i = 0; i < nSensors; i++)
	{
		sensor_list_.push_back(sensorFac.createPclOpenNI2Grabber());
		detected_sphere_[sensor_list_.back()->getId()].setResolution(6);
		calib_positions_[sensor_list_.back()->getId()] = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl::PointCloud<pcl::PointXYZ>());
	}

	// ui setup
	calibResetBtn_.addListener(this, &ofApp::reset_calibration);
	performEstimBtn_.addListener(this, &ofApp::performICPTransformationEstimation);

	ui_.setup();
	ui_.add(bgSl_.setup(background_));
	ui_.add(resolutionSl_.setup(resolution_));
	ui_.add(passMinSl_.setup(passMin_));
	ui_.add(passMaxSl_.setup(passMax_));
	ui_.add(minSl_.setup(min_));
	ui_.add(maxSl_.setup(max_));
	ui_.add(samplesSl_.setup(samples_));
	ui_.add(errorSl_.setup(error_));
	ui_.add(percentSl_.setup(percent_));
	ui_.add(meanSampleSl_.setup(meanSamples_));
	ui_.add(calibResetBtn_.setup("Reset Calibration"));
	ui_.add(performEstimBtn_.setup("Perform Calibration"));

	cam_.rotate(180, 1, 0, 0);
	cam_.setFarClip(100000);
}

//--------------------------------------------------------------
void ofApp::update()
{
	bool take_snapshot = true;
	// for each sensor
	for (auto& sensor : sensor_list_)
	{
		// get current point cloud
		auto cloud = sensor->getCloudSource()->getOutputCloud();
		if (cloud != nullptr)
		{
			// downsample cloud for searching sphere
			recon::CloudPtr cloud_downsampled(new recon::Cloud());
			downsample(cloud, cloud_downsampled);

			// remove background
			recon::CloudPtr cloud_wo_back(new recon::Cloud());
			removeBackground(cloud_downsampled, cloud_wo_back);

			// find sphere
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
			Eigen::VectorXf s_param;

			findSphere(cloud_wo_back, inliers, s_param);
			auto sphere_r = s_param.w();
			auto sphere_x = s_param.x();
			auto sphere_y = s_param.y();
			auto sphere_z = s_param.z();

			if (inliers->indices.size() > 0)
			{
				sphere_detected_[sensor->getId()] = true;

				// get in- and outlier cloud
				recon::CloudPtr in_cloud(new recon::Cloud());
				recon::CloudPtr out_cloud(new recon::Cloud());
				extractInOutliers(cloud_wo_back, inliers, in_cloud, out_cloud);
				createOfMeshFromPoints(in_cloud, ofColor(0, 255, 0, 255), inliers_mesh_[sensor->getId()]);
				auto c = cloudColors[sensor->getId()];
				c.a = 64;
				createOfMeshFromPoints(out_cloud, c, mesh_map_[sensor->getId()]);

				// calculate mean radius of calib target
				meanR_[sensor->getId()] = approxRollingAverage(meanR_[sensor->getId()], sphere_r * 1000, meanSamples_);
				detected_sphere_[sensor->getId()].setRadius(meanR_[sensor->getId()]);

				// remember last mean position
				last_mean_pos_[sensor->getId()] = ofVec3f(meanX_[sensor->getId()], meanY_[sensor->getId()], meanZ_[sensor->getId()]);

				// calculate new mean position
				meanX_[sensor->getId()] = approxRollingAverage(meanX_[sensor->getId()], sphere_x * 1000, meanSamples_);
				meanY_[sensor->getId()] = approxRollingAverage(meanY_[sensor->getId()], sphere_y * 1000, meanSamples_);
				meanZ_[sensor->getId()] = approxRollingAverage(meanZ_[sensor->getId()], sphere_z * 1000, meanSamples_);
				detected_sphere_location_[sensor->getId()].set(meanX_[sensor->getId()], meanY_[sensor->getId()], meanZ_[sensor->getId()]);
			}
			else
			{
				sphere_detected_[sensor->getId()] = false;
				// make ofMesh for displaying
				ofMesh mesh;
				auto c = cloudColors[sensor->getId()];
				c.a = 64;
				createOfMeshFromPoints(cloud_wo_back, c, mesh);
				mesh_map_.erase(sensor->getId());
				mesh_map_.insert(std::pair<int, ofMesh>(sensor->getId(), mesh));
			}

			// when tracking target has moved more than a cm in one of the directions
			if (std::fabs(last_mean_pos_[sensor->getId()].x - meanX_[sensor->getId()]) >= 15
				|| std::fabs(last_mean_pos_[sensor->getId()].y - meanY_[sensor->getId()]) >= 15
				|| std::fabs(last_mean_pos_[sensor->getId()].z - meanZ_[sensor->getId()]) >= 15)
			{
				take_snapshot = false;
			}
		}
	}
	// take snapshot if conditions where met
	if (take_snapshot)
	{
		// check elapsed time in static pose
		auto now = std::chrono::steady_clock::now();
		auto time_static = now - static_since_;
		auto time_since_last_snap = now - last_snap_;
		//		std::cout << "Static time: " << std::chrono::duration_cast<std::chrono::seconds, long long, std::nano>(time_static).count()
		//			<< "Time since last snap: " << std::chrono::duration_cast<std::chrono::seconds, long long, std::nano>(time_since_last_snap).count() << std::endl;
		// if bigger than time to snapshot, do snapshot
		if (time_static >= static_time_to_snapshot_ && time_since_last_snap >= static_time_to_snapshot_)
		{
			for (auto& sensor : sensor_list_)
			{
				pcl::PointXYZ calib_point(detected_sphere_location_[sensor->getId()].x,
				                          detected_sphere_location_[sensor->getId()].y,
				                          detected_sphere_location_[sensor->getId()].z);
				calib_positions_[sensor->getId()]->push_back(calib_point);
				last_snap_ = std::chrono::steady_clock::now();
				performICPTransformationEstimation();
			}
		}
	}
	else
	{
		// reset timer
		static_since_ = std::chrono::steady_clock::now();
	}
}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofBackground(background_);

	ofDrawBitmapString("fps: " + std::to_string(ofGetFrameRate()) + " calpos: " + std::to_string(calib_positions_[0]->size()), 10, 10);

	cam_.begin();
	ofEnableDepthTest();
	for (auto& sensor : sensor_list_)
	{
		ofPushMatrix();
		auto ext = sensor->getDepthExtrinsics();
		auto translation = toOfVector3(*ext->getTranslation());
		auto rotation = toOfQuaternion(*ext->getRotation());
		ofVec3f qaxis;
		float qangle;
		rotation.getRotate(qangle, qaxis);
		ofTranslate(translation);
		ofRotate(qangle, qaxis.x, qaxis.y, qaxis.z);

		mesh_map_[sensor->getId()].drawVertices();

		if (sphere_detected_[sensor->getId()])
		{
			inliers_mesh_[sensor->getId()].draw();

			ofPushMatrix();
			ofPushStyle();
			ofSetColor(255, 0, 0);
			ofTranslate(detected_sphere_location_[sensor->getId()]);
			detected_sphere_[sensor->getId()].drawWireframe();
			ofPopMatrix();
			ofPopStyle();
		}

		for (auto& p : *calib_positions_[sensor->getId()])
		{
			ofPushStyle();
			ofSetColor(cloudColors[sensor->getId()]);
			ofDrawBox(p.x, p.y, p.z, 30);
			ofPopStyle();
		}
		ofPopMatrix();
	}


	cam_.end();

	ofDisableDepthTest();
	ui_.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key)
{
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg)
{
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo)
{
}

void ofApp::reset_calibration()
{
	for (auto& sensor : sensor_list_)
	{
		calib_positions_[sensor->getId()]->clear();
	}
}

float ofApp::approxRollingAverage(float avg, float new_sample, int window)
{
	avg -= avg / window;
	avg += new_sample / window;

	return avg;
}

void ofApp::downsample(recon::CloudConstPtr src, recon::CloudPtr trgt)
{
	pcl::VoxelGrid<recon::PointType> sor;
	sor.setInputCloud(src);
	sor.setLeafSize(resolution_, resolution_, resolution_);
	sor.filter(*trgt);
}

void ofApp::removeBackground(recon::CloudPtr src, recon::CloudPtr trgt)
{
	pcl::PassThrough<recon::PointType> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(passMin_, passMax_);
	pass.setInputCloud(src);
	pass.filter(*trgt);
}

void ofApp::findSphere(recon::CloudPtr src, pcl::PointIndices::Ptr inliers, Eigen::VectorXf& sphereParam)
{
	sphere_model_.reset(new pcl::SampleConsensusModelSphere<recon::PointType>(src));
	sphere_model_->setRadiusLimits(min_, max_);
	pcl::RandomSampleConsensus<recon::PointType> ransac(sphere_model_);

	ransac.setDistanceThreshold(error_);
	ransac.setMaxIterations(samples_);
	ransac.setProbability(percent_);
	ransac.computeModel();

	ransac.getInliers(inliers->indices);

	ransac.getModelCoefficients(sphereParam);
}

void ofApp::extractInOutliers(recon::CloudPtr src, pcl::PointIndices::Ptr inliers, recon::CloudPtr in_cloud, recon::CloudPtr out_cloud)
{
	pcl::ExtractIndices<recon::PointType> extract;
	extract.setInputCloud(src);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*in_cloud);

	extract.setNegative(true);
	extract.filter(*out_cloud);
}

void ofApp::performICPTransformationEstimation()
{
	// perform icp on calib_position pointclouds
	std::list<recon::AbstractSensor::Ptr>::iterator it;
	std::list<recon::AbstractSensor::Ptr>::iterator ref = sensor_list_.begin();
	for (it = sensor_list_.begin(); it != sensor_list_.end(); ++it)
	{
		auto sensor1 = *ref;
		auto sensor2 = *it;
		if (sensor1 != sensor2)
		{
			auto cloud1 = calib_positions_[sensor1->getId()];
			auto cloud2 = calib_positions_[sensor2->getId()];

			pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
			icp.setInputCloud(cloud2);
			icp.setInputTarget(cloud1);
			icp.setMaxCorrespondenceDistance(1000);
			icp.setMaximumIterations(100);
			pcl::PointCloud<pcl::PointXYZ> cloud_reg;
			icp.align(cloud_reg);


			auto trans = Eigen::Affine3f(icp.getFinalTransformation());
			Eigen::Vector4f t(trans.translation().x()/1000, trans.translation().y() / 1000, trans.translation().z() / 1000, 0);
			Eigen::Quaternionf r = Eigen::Quaternionf(trans.rotation());
			recon::CameraExtrinsics::Ptr ext(new recon::CameraExtrinsics(t, r));

			sensor2->setDepthExtrinsics(ext);
		}
	}
}





