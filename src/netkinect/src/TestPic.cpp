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
	int ret = 0;

	Mat* video_frame;
	char* video_data;
	Mat* depth_frame;
	char* depth_data;

	KinectFrameMessage frame_message;
	KinectFrameMessage tes;

	namedWindow("rgb",CV_WINDOW_AUTOSIZE);
	namedWindow("depth",CV_WINDOW_AUTOSIZE);
	while(1){
		if ((ret = kinect.getData(VIDEO, &video_data)) != 0){
			LOG_WARNING << "could not receive video frame from kinect" << endl;
			continue;
		}
		if ((ret = kinect.getData(DEPTH, &depth_data)) != 0){
			LOG_WARNING << "could not receive depth frame from kinect" << endl;
			continue;
		}

		/*frame_message.set_video_data((void*) video_image, VIDEO_FRAME_MAX_SIZE);
		frame_message.set_depth_data((void*) depth_image, DEPTH_FRAME_MAX_SIZE);

		frame_message.SerializeToArray(send_data, frame_message.ByteSize());

		tes.ParseFromArray(send_data);*/

		video_frame = new Mat(Size(640, 480), CV_8UC3, video_data);
		cvtColor(*video_frame, *video_frame, CV_RGB2BGR);


		char* depth = (char*) malloc(DEPTH_FRAME_MAX_SIZE);
		memcpy(depth, depth_data, DEPTH_FRAME_MAX_SIZE);

		depth_frame = new Mat(Size(640, 480), CV_16UC1, depth);

		Mat depthf (Size(640, 480), CV_8UC1);
		depth_frame->convertTo(depthf, CV_8UC1, 255.0/2048.0);

		imshow("rgb", *video_frame);
		imshow("depth", depthf);

		cvWaitKey(10);

		delete video_frame;
		delete depth_frame;
		free(depth);
	}


	return EXIT_SUCCESS;
}
