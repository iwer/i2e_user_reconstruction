// Be sure to link with -lfreenect_sync
#include <stdlib.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "libfreenect/libfreenect_sync.h"

#include "../gen/KinectFrameMessage.pb.h"
#include "KinectWrapper.h"
#include "Logger.h"


using namespace cv;
using namespace std;

int main(){
	KinectWrapper kinect = KinectWrapper::getInstance();

	uint8_t* rgb;
	uint16_t* depth;

	int ret = 0;

	Mat depthMat(Size(640,480),CV_16UC1);
	Mat depthf (Size(640,480),CV_8UC1);
	Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));

	namedWindow("rgb",CV_WINDOW_AUTOSIZE);
	namedWindow("depth",CV_WINDOW_AUTOSIZE);
	while(1){
		if ((ret = kinect.getData(VIDEO, (void**) &rgb)) != 0){
			LOG_WARNING << "could not receive video frame from kinect" << endl;
			continue;
		}
		if ((ret = kinect.getData(DEPTH, (void**) &depth)) != 0){
			LOG_WARNING << "could not receive depth frame from kinect" << endl;
			continue;
		}


		rgbMat.data = rgb;
		depthMat.data = (uchar*) depth;

		cvtColor(rgbMat, rgbMat, CV_RGB2BGR);

		depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);

		cv::imshow("rgb", rgbMat);
		//cv::imshow("depth", depthf);
	}


	return EXIT_SUCCESS;
}
