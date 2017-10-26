#include "ofApp.h"
#include "NetKinectSensor.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <common/common.h>
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
//--------------------------------------------------------------
void ofApp::setup()
{
	font.load("verdana.ttf", 28);


	std::chrono::seconds five_sec(3);
	static_time_to_snapshot_ = five_sec;
	cloudColors[0].set(255, 0, 0);
	cloudColors[1].set(0, 255, 0);
	cloudColors[2].set(0, 0, 255);
	cloudColors[3].set(255, 255, 0);

    // KINECT SERVER ########
    std::cout << "Waiting for networked Kinect servers to connect" << std::endl;
    while(!netkinect_api_.isAbleToDeliverData()) {
        std::cout << ".";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << std::endl << "Done. " << netkinect_api_.getClientCount() << " networked Kinects connected." << std::endl;
	// SENSORS ##############
	recon::SensorFactory sensorFac;

	auto nSensors = netkinect_api_.getClientCount();
	for (int i = 0; i < nSensors; i++)
	{
        recon::AbstractSensor::Ptr sensor = boost::shared_ptr<recon::AbstractSensor>(new NetKinectSensor(netkinect_api_.getClient(i), i));
        sensor_list_.push_back(sensor);

        detected_sphere_[sensor_list_.back()->getId()].setResolution(6);
				calib_positions_[sensor_list_.back()->getId()] = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl::PointCloud<pcl::PointXYZ>());
				recon::CloudPtr newcloud(new recon::Cloud());
				cloud_map_[sensor_list_.back()->getId()] = newcloud;
	}

	// UI ############
	ui_.setup();
	ui_.setSize(300, ofGetHeight());
	ui_.setDefaultWidth(300);
	ui_.setWidthElements(300);

	ui_.add(bgSl_.setup(background_));

	calibResetBtn_.addListener(this, &ofApp::reset_calibration);
	performEstimBtn_.addListener(this, &ofApp::performICPTransformationEstimation);
	ui_.add(calibResetBtn_.setup("Reset Calibration"));
	ui_.add(performEstimBtn_.setup("Perform Calibration"));

	saveCalibrationBtn_.addListener(this, &ofApp::saveCalibrationToFile);
	loadCalibrationBtn_.addListener(this, &ofApp::loadCalibrationFromFile);
	ui_.add(saveCalibrationBtn_.setup("Save calibration"));
	ui_.add(loadCalibrationBtn_.setup("Load calibration"));

	globalPositionParams_.setName("Global Position Parameters");
	globalPositionParams_.add(globalCalibration_);
	globalPositionParams_.add(xTrans_);
	globalPositionParams_.add(yTrans_);
	globalPositionParams_.add(zTrans_);
	globalPositionParams_.add(xRot_);
	globalPositionParams_.add(yRot_);
	globalPositionParams_.add(zRot_);
	ui_.add(globalPositionParams_);

	filteringParams_.setName("Filtering Parameters");
	filteringParams_.add(resolution_);
	filteringParams_.add(passMin_);
	filteringParams_.add(passMax_);
	ui_.add(filteringParams_);

	trackingParams_.setName("Tracking Parameters");
	trackingParams_.add(trackingEnabled_);
	trackingParams_.add(minR_);
	trackingParams_.add(maxR_);
	trackingParams_.add(samples_);
	trackingParams_.add(error_);
	trackingParams_.add(percent_);
	trackingParams_.add(meanSamples_);
	trackingParams_.add(movementThreshold_);
	ui_.add(trackingParams_);

	iaParams_.setName("SAC-IA Parameters");
	iaParams_.add(iaMinSampleDistance__);
	iaParams_.add(iaMaxCorrespondenceDistance_);
	iaParams_.add(iaIterations_);
	ui_.add(iaParams_);

	icpParams_.setName("ICP Parameters");
	icpParams_.add(icpDistanceThreshold_);
	icpParams_.add(icpIterations_);
	ui_.add(icpParams_);

	// CAMERA ##############

	cam_.rotate(180, 0, 1, 0);
	cam_.setFarClip(100000);
	cam_.enableMouseInput();
	cam_.disableMouseMiddleButton();

	//player_.load("click.mp3");
}

//--------------------------------------------------------------
void ofApp::update()
{
	if (globalCalibration_) {
		// update calibration of first sensor, the one all others align to, for global positioning
		ofVec3f x(1, 0, 0), y(0, 1, 0), z(0, 0, 1);
		ofQuaternion rquat;
		rquat.makeRotate(xRot_, x, yRot_, y, zRot_, z);
		auto pos = ofVec3f(xTrans_ * 1000, yTrans_ * 1000, zTrans_ * 1000);
		Eigen::Vector4f translation;
		toEigenVector4f(pos, translation);
		auto rotation = toEigenQuaternionf(rquat);
		recon::CameraExtrinsics::Ptr ext(new recon::CameraExtrinsics(translation, rotation));
		sensor_list_.front()->setDepthExtrinsics(ext);
	}

	if(netkinect_api_.isAbleToDeliverData()) {
		for (int i = 0; i < netkinect_api_.getClientCount(); i++) {
			    cloudsize[i] = netkinect_api_.getClient(i)->getCloud(&clouddata[i], cloudsize[i], cloud_map_[i]);

		    //Encode cloud into pcl::PointCloud<pcl::PointXYZ>
				/*
		    recon::CloudPtr newcloud(new recon::Cloud());
				newcloud->resize(cloudsize[i]/3);
				{
					pcl::ScopeTime loop("Loop");
					recon::PointType p;

			    for (int j = 0; j <= cloudsize[i] - 3; j += 3) {
							p.z = clouddata[i][j];
							p.x = clouddata[i][j + 1];
							p.y = -clouddata[i][j + 2];

							//if(p.x != 0 || p.y != 0 || p.z != 0) {
							//	std::cout << p.x << " " << p.y << " " << p.z <<std::endl;
							//}

							newcloud->push_back(p);
			    }
			  }
		    // save for collection
		    cloud_map_[i] = newcloud; */
				netkinect_api_.getClient(i)->processedData();
		}
	}

	bool take_snapshot = true;
	// for each sensor
	for (auto& sensor : sensor_list_)
	{
		// get current point cloud, points are in camera space (x left, y up, z back)
		//auto cloud = sensor->getCloudSource()->getOutputCloud();
		auto cloud = cloud_map_[sensor->getId()];
		if (cloud != nullptr)
		{
			// downsample cloud for searching sphere
			recon::CloudPtr cloud_downsampled(new recon::Cloud());
			downsample(cloud, cloud_downsampled, resolution_);

			// remove background
			recon::CloudPtr cloud_wo_back(new recon::Cloud());
			removeBackground(cloud_downsampled, cloud_wo_back, passMin_, passMax_, false);

			// find sphere, still in camera space
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
			if (trackingEnabled_)
			{
				Eigen::VectorXf s_param;

				findSphere(cloud_wo_back, inliers, s_param);

				if (inliers->indices.size() > 0 && s_param.size() == 4)
				{
					auto sphere_r = s_param.w();
					auto sphere_x = s_param.x();
					auto sphere_y = s_param.y();
					auto sphere_z = s_param.z();

					sphere_detected_[sensor->getId()] = true;

					// get in- and outlier cloud
					recon::CloudPtr in_cloud(new recon::Cloud());
					recon::CloudPtr out_cloud(new recon::Cloud());
					extractInOutliers(cloud_wo_back, inliers, in_cloud, out_cloud);
					// inliers mesh to draw
					createOfMeshFromPoints(in_cloud, ofColor(0, 255, 0, 255), inliers_mesh_[sensor->getId()]);
					auto c = cloudColors[sensor->getId()];
					c.a = 64;
					// outliers mesh to draw
					createOfMeshFromPoints(out_cloud, c, mesh_map_[sensor->getId()]);

					// calculate mean radius of calib target
					meanR_[sensor->getId()] = approxRollingAverage(meanR_[sensor->getId()], sphere_r * 1000, meanSamples_);
					detected_sphere_[sensor->getId()].setRadius(meanR_[sensor->getId()]);

					// calculate new mean position (in camera space)
					last_mean_pos_[sensor->getId()] = ofVec3f(meanX_[sensor->getId()], meanY_[sensor->getId()], meanZ_[sensor->getId()]);
					meanX_[sensor->getId()] = approxRollingAverage(meanX_[sensor->getId()], sphere_x * 1000, meanSamples_);
					meanY_[sensor->getId()] = approxRollingAverage(meanY_[sensor->getId()], sphere_y * 1000, meanSamples_);
					meanZ_[sensor->getId()] = approxRollingAverage(meanZ_[sensor->getId()], sphere_z * 1000, meanSamples_);
					detected_sphere_location_[sensor->getId()].set(meanX_[sensor->getId()], meanY_[sensor->getId()], meanZ_[sensor->getId()]);
				}
				// if no sphere was found
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
				if (std::fabs(last_mean_pos_[sensor->getId()].x - meanX_[sensor->getId()]) >= movementThreshold_
					|| std::fabs(last_mean_pos_[sensor->getId()].y - meanY_[sensor->getId()]) >= movementThreshold_
					|| std::fabs(last_mean_pos_[sensor->getId()].z - meanZ_[sensor->getId()]) >= movementThreshold_)
				{
					take_snapshot = false;
				}
			}
			// if tracking is disabled
			else
			{
				sphere_detected_[sensor->getId()] = false;
				// make ofMesh for displaying
				ofMesh mesh;
				createOfMeshFromPoints(cloud_wo_back, mesh);
				mesh_map_.erase(sensor->getId());
				mesh_map_.insert(std::pair<int, ofMesh>(sensor->getId(), mesh));
			}
		}
	}
	// take snapshot if conditions where met
	if (take_snapshot && trackingEnabled_)
	{
		// check elapsed time in static pose
		auto now = std::chrono::steady_clock::now();
		auto time_static = now - static_since_;
		auto time_since_last_snap = now - last_snap_;

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

			//player_.play();
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

	font.drawString(icpFitnessScore_, ofGetWidth() - 200, 50);


	cam_.begin();
	ofEnableDepthTest();

	if(globalCalibration_)
	{
		ofDrawAxis(100);

	}

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

		if (trackingEnabled_)
		{
			if (sphere_detected_[sensor->getId()])
			{
				inliers_mesh_[sensor->getId()].draw();

				ofPushMatrix();
				ofPushStyle();
				ofSetColor(cloudColors[sensor->getId()]);
				ofTranslate(detected_sphere_location_[sensor->getId()]);
				detected_sphere_[sensor->getId()].drawWireframe();
				ofPopMatrix();
				ofPopStyle();
			}
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

void ofApp::findSphere(recon::CloudPtr src, pcl::PointIndices::Ptr inliers, Eigen::VectorXf& sphereParam)
{
	if (src->size() > 0)
	{
		sphere_model_.reset(new pcl::SampleConsensusModelSphere<recon::PointType>(src));
		sphere_model_->setRadiusLimits(minR_, maxR_);

		pcl::RandomSampleConsensus<recon::PointType> ransac(sphere_model_);

		ransac.setDistanceThreshold(error_);
		ransac.setMaxIterations(samples_);
		ransac.setProbability(percent_);
		ransac.computeModel();


		// obtain results

		ransac.getInliers(inliers->indices);
		ransac.getModelCoefficients(sphereParam);
	}
}

void ofApp::computeFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr refCloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr refFeatures)
{
	pcl::PointCloud<pcl::Normal>::Ptr refNormals(new pcl::PointCloud<pcl::Normal>());
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method(new pcl::search::KdTree<pcl::PointXYZ>());
	norm_est.setInputCloud(refCloud);
	norm_est.setSearchMethod(search_method);
	norm_est.setRadiusSearch(.5);
	norm_est.compute(*refNormals);

	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	fpfh_est.setInputCloud(refCloud);
	fpfh_est.setInputNormals(refNormals);
	fpfh_est.setSearchMethod(search_method);
	fpfh_est.setRadiusSearch(1);
	fpfh_est.compute(*refFeatures);
}

void ofApp::performICPTransformationEstimation()
{
	// perform icp on calib_position pointclouds
	std::list<recon::AbstractSensor::Ptr>::iterator it;
	std::list<recon::AbstractSensor::Ptr>::iterator ref = sensor_list_.begin();

	// transform points of first sensor to global position
	auto sensor1 = *ref;
	auto refCloud = calib_positions_[sensor1->getId()];
	pcl::PointCloud<pcl::PointXYZ>::Ptr refCloudGlobalTransformed(new pcl::PointCloud<pcl::PointXYZ>());


	// this point cloud is in of space, so translation has to be scaled up
	auto trnsrfrm = sensor1->getDepthExtrinsics()->getTransformation();
	trnsrfrm.translation() *= 1000;
	pcl::transformPointCloud(*refCloud, *refCloudGlobalTransformed, trnsrfrm);

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr refFeatures(new pcl::PointCloud<pcl::FPFHSignature33>());
	computeFeatures(refCloudGlobalTransformed, refFeatures);


	for (it = sensor_list_.begin(); it != sensor_list_.end(); ++it)
	{
		// estimate transformation from each sensor to sensor1
		auto sensor2 = *it;
		if (sensor1 != sensor2)
		{
			auto cloud2 = calib_positions_[sensor2->getId()];
			if (refCloudGlobalTransformed->size() >= 3 && cloud2->size() >= 3)
			{
				pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud2Features(new pcl::PointCloud<pcl::FPFHSignature33>());
				computeFeatures(cloud2, cloud2Features);

				pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
				sac_ia.setMinSampleDistance(iaMinSampleDistance__);
				sac_ia.setMaxCorrespondenceDistance(iaMaxCorrespondenceDistance_ * 1000);
				sac_ia.setMaximumIterations(iaIterations_);
				sac_ia.setInputTarget(refCloudGlobalTransformed);
				sac_ia.setTargetFeatures(refFeatures);
				//sac_ia.setInputCloud(cloud2); deprecated
				sac_ia.setInputSource(cloud2);
				sac_ia.setSourceFeatures(cloud2Features);

				pcl::PointCloud<pcl::PointXYZ>::Ptr registration_output(new pcl::PointCloud<pcl::PointXYZ>());
				sac_ia.align(*registration_output);
				auto ia_alignment = sac_ia.getFinalTransformation();

				pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float> icp;
				//icp.setInputCloud(registration_output);
                icp.setInputSource(registration_output);
				icp.setInputTarget(refCloudGlobalTransformed);
				icp.setMaxCorrespondenceDistance(icpDistanceThreshold_ * 1000);
				icp.setMaximumIterations(icpIterations_);
				//icp.setTransformationEpsilon(1e-6);
				//icp.setEuclideanFitnessEpsilon(1);

				pcl::PointCloud<pcl::PointXYZ> cloud_reg;
				icp.align(cloud_reg);

				//std::cout << "ICP has converged:" << icp.hasConverged() << " score: " <<
				//	icp.getFitnessScore() << std::endl;
				icpFitnessScore_ = std::to_string(icp.getFitnessScore());

				auto transformation = Eigen::Affine3f(ia_alignment * icp.getFinalTransformation());
				Eigen::Vector4f t(transformation.translation().x() / 1000, transformation.translation().y() / 1000, transformation.translation().z() / 1000, 0);
				auto r = Eigen::Quaternionf(transformation.rotation());
				recon::CameraExtrinsics::Ptr ext(new recon::CameraExtrinsics(t, r));

				sensor2->setDepthExtrinsics(ext);
			}
		}
	}
}

void ofApp::loadCalibrationFromFile()
{
	SensorCalibrationSettings set;
	for (auto& s : sensor_list_)
	{
		set.loadCalibration(s, s->getId());
	}
}

void ofApp::saveCalibrationToFile()
{
	SensorCalibrationSettings set;

	for (auto& s : sensor_list_)
	{
		set.saveCalibration(s, s->getId());
	}
}
