#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "recon/SensorFactory.h"
#include "of-pcl-bridge/of-pcl-bridge.h"
#include "recon/typedefs.h"
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <chrono>

class ofApp : public ofBaseApp{

	public:
		ofApp()
			: min_("Min Radius", .1, .01, 1)
			, max_("Max Radius", .3, .1, 1)
			, error_("Modell Error", .01, .001, .2)
			, percent_("Inlier %", .99, .2, 1)
			, resolution_("Resolution", .03, .005, .1)
			, samples_("# Samples", 2500, 10, 5000)
			, passMin_("Z Min", .01, .01, 8)
			, passMax_("Z Max", 2.5, .01, 8)
			, meanSamples_("Smoothing Frame", 1, 1, 60)
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
		
		float approxRollingAverage(float avg, float new_sample, int window);
		void reset_calibration();
		std::list<recon::AbstractSensor::Ptr> sensor_list_;

		ofEasyCam cam_;

		std::map<int, ofMesh> mesh_map_;
		std::map<int, ofMesh> inliers_mesh_;

		pcl::SampleConsensusModelSphere<recon::PointType>::Ptr sphere_model_;

		std::map<int, bool> sphere_detected_;
		std::map<int, float> meanX_;
		std::map<int, float> meanY_;
		std::map<int, float> meanZ_;
		std::map<int, float> meanR_;

		std::map<int, ofVec3f> last_mean_pos_;

		std::map<int, ofVec3f> detected_sphere_location_;
		std::map<int, ofSpherePrimitive> detected_sphere_;

		std::map<int, pcl::PointCloud<pcl::PointXYZ>> calib_positions_;

		std::chrono::duration<long long, std::nano> static_time_to_snapshot_;
		std::chrono::time_point<std::chrono::steady_clock> static_since_;

		// gui stuff
		ofParameter<float> min_;
		ofParameter<float> max_;
		ofParameter<float> error_;
		ofParameter<float> percent_;
		ofParameter<float> resolution_;
		ofParameter<int> samples_;
		ofParameter<float> passMin_;
		ofParameter<float> passMax_;
		ofParameter<int> meanSamples_;

		ofxPanel ui_;
		ofxFloatSlider minSl_;
		ofxFloatSlider maxSl_;
		ofxFloatSlider errorSl_;
		ofxFloatSlider percentSl_;
		ofxFloatSlider resolutionSl_;
		ofxIntSlider samplesSl_;
		ofxFloatSlider passMinSl_;
		ofxFloatSlider passMaxSl_;
		ofxIntSlider meanSampleSl_;
		ofxButton calibResetBtn_;
};
