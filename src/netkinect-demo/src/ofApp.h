#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "recon/SensorFactory.h"
#include "of-pcl-bridge/of-pcl-bridge.h"
#include "recon/typedefs.h"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include "common/SensorCalibrationSettings.h"
#include <chrono>
#include <ServerAPI.h>

class ofApp : public ofBaseApp{

	public:
		ofApp()
			: minR_("Min Radius", .1, .01, 1)
			, maxR_("Max Radius", .3, .1, 1)
			, trackingEnabled_("Enable Tracking", false)
			, globalCalibration_("Global Calibration", true)
			, error_("Modell Error", .01, .001, .2)
			, percent_("Inlier %", .99, .2, 1)
			, resolution_("Resolution", .03, .005, .1)
			, samples_("# Samples", 10000, 10, 50000)
			, passMin_("Z Min", .01, .01, 8)
			, passMax_("Z Max", 2.5, .01, 8)
			, meanSamples_("Smoothing Frame", 3, 1, 60)
			, background_("Background", 127, 0, 255)
			, movementThreshold_("Movement threshold", 15, .1, 50)
			, icpDistanceThreshold_("Distance Threshold ", 2, .001, 3)
			, icpIterations_("# Iterations", 500, 20, 1000)
			, iaMinSampleDistance__("Min Sample Distance", .05, .001, 1)
			, iaMaxCorrespondenceDistance_("Max correspondence distance", 3, .1, 8)
			, iaIterations_("Max Iterations", 10000, 10, 50000)
			, xTrans_("X Translation", 0, -5, 5)
			, yTrans_("Y Translation", 0, -5, 5)
			, zTrans_("Z Translation", 0, -5, 5)
			, xRot_("X Rotation", 0, -180, 180)
			, yRot_("Y Rotation", 0, -180, 180)
			, zRot_("Z Rotation", 0, -180, 180)
		{}

		void setup();
		void update();
		void draw();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void mouseEntered(int x, int y);
		void mouseExited(int x, int y);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		
		
		void reset_calibration();
		void findSphere(recon::CloudPtr src, pcl::PointIndices::Ptr inliers, Eigen::VectorXf &sphereParam);
		void computeFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr refCloud, pcl::PointCloud<pcl::FPFHSignature33>::Ptr refFeatures);
		void performICPTransformationEstimation();
		
		void loadCalibrationFromFile();
		void saveCalibrationToFile();

		ofSoundPlayer player_;

		ofEasyCam cam_;

		// sensor stuff
		ofColor cloudColors[4];
		std::list<recon::AbstractSensor::Ptr> sensor_list_;
		std::map<int, ofMesh> mesh_map_;
		std::map<int, ofMesh> inliers_mesh_;

		// sphere tracking stuff
		pcl::SampleConsensusModelSphere<recon::PointType>::Ptr sphere_model_;

		std::map<int, bool> sphere_detected_;
		// sphere coordinates in camera space of associated sensor
		std::map<int, float> meanX_;
		std::map<int, float> meanY_;
		std::map<int, float> meanZ_;
		std::map<int, float> meanR_;
		std::map<int, ofVec3f> last_mean_pos_;
		std::map<int, ofVec3f> detected_sphere_location_;
		std::map<int, ofSpherePrimitive> detected_sphere_;

		// calibration snapshots of sphere position in camera space
		std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> calib_positions_;

		// time variables for snapshot intervallometer
		std::chrono::duration<long long, std::nano> static_time_to_snapshot_;
		std::chrono::time_point<std::chrono::steady_clock> static_since_;
		std::chrono::time_point<std::chrono::steady_clock> last_snap_;

		// NetKinect
        ServerAPI netkinect_api_;

		// gui stuff
		ofTrueTypeFont font;
		std::string icpFitnessScore_;

		ofParameter<float> background_;
		ofParameter<float> minR_;
		ofParameter<float> maxR_;
		
		ofParameterGroup globalPositionParams_;
		ofParameter<bool> globalCalibration_;
		ofParameter<float> xTrans_;
		ofParameter<float> yTrans_;
		ofParameter<float> zTrans_;
		ofParameter<float> xRot_;
		ofParameter<float> yRot_;
		ofParameter<float> zRot_;

		ofParameterGroup trackingParams_;
		ofParameter<bool> trackingEnabled_;
		ofParameter<int> samples_;
		ofParameter<float> error_;
		ofParameter<float> percent_;
		ofParameter<int> meanSamples_;
		ofParameter<float> movementThreshold_;

		ofParameterGroup filteringParams_;
		ofParameter<float> resolution_;
		ofParameter<float> passMin_;
		ofParameter<float> passMax_;

		ofParameterGroup iaParams_;
		ofParameter<float> iaMinSampleDistance__;
		ofParameter<float> iaMaxCorrespondenceDistance_;
		ofParameter<int> iaIterations_;

		ofParameterGroup icpParams_;
		ofParameter<float> icpDistanceThreshold_;
		ofParameter<int> icpIterations_;

		ofxPanel ui_;
		ofxFloatSlider bgSl_;
		ofxButton calibResetBtn_;
		ofxButton performEstimBtn_;
		ofxButton saveCalibrationBtn_;
		ofxButton loadCalibrationBtn_;
};
