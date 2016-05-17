#pragma once

#include "ofMain.h"
#include <recon/OpenNI2Sensor.h>

class ofApp : public ofBaseApp{

	public:
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
		
		void cloud_cb1(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);
		void cloud_cb2(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud);

		ofEasyCam cam_;

		pcl::io::OpenNI2Grabber * sensor1_;
		pcl::io::OpenNI2Grabber * sensor2_;

		std::mutex cond_lock_;
		std::condition_variable cond_;

		ofMesh mesh1;
		ofMesh mesh2;
};
