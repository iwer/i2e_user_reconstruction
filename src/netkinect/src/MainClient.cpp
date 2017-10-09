#include <iostream>
#include <ratio>
#include <chrono>
#include <ctime>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>

#include "Common.h"

#include "../gen/KinectFrameMessage.pb.h"
#include "KinectWrapper.h"
#include "Server.h"
#include "Sync.h"
#include "Logger.h"
#include "PCLUtil.h"


#define LOG_LEVEL DEBUG

using namespace std;
using namespace chrono;

volatile bool running = true;
void signalHandler(int signal){
	if (signal == SIGINT
	 || signal == SIGTERM
	 || signal == SIGQUIT){
		running = false;
	}
}

int main(){
	KinectWrapper kinect = KinectWrapper::getInstance();

	if (getuid()){
		cout << "You have to have root permission! Try sudo." << endl;
		kinect.setLed(LED_RED);
		return -1;
	}

	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);
	signal(SIGQUIT, signalHandler);

	SET_LOG_LEVEL(LOG_LEVEL);

	kinect.setLed(LED_YELLOW);

	char* video_image;
	char* depth_image;

	string video_string;
	video_string.resize(VIDEO_FRAME_MAX_SIZE);

	string depth_string;
	depth_string.resize(DEPTH_FRAME_MAX_SIZE);

  	KinectFrameMessage frame_message;
	int id = 0;
	int is_leader = 0;


	/*
		the first one always takes ~10 times the normal time (usb handshake,
		resync etc.), so just do it once before the "real" program starts
	*/
	LOG_DEBUG << "handle usb handshake..." << endl;
	cout << "Initialize Kincet.." << endl;
	kinect.handleUSBHandshake();

	Sync sync;
	is_leader = sync.connect();

	Server server;

	if ((id = server.connect(is_leader)) == -1){
		kinect.setLed(LED_RED);
		return -1;
	}


	time_t timestamp = 0;

	cout << "Sending data to server.." << endl;
	kinect.setLed(LED_GREEN);
	while(running){
		if (server.isClosed()){
			break;
		}

		//LOG_DEBUG << "trying to get frame from kinect" << endl;

		if (kinect.getData(VIDEO, &video_image) != 0){
			LOG_WARNING << "could not receive video frame from kinect" << endl;
			continue;
		}
		if (kinect.getData(DEPTH, &depth_image) != 0){
			LOG_WARNING << "could not receive depth frame from kinect" << endl;
			continue;
		}

		timestamp = system_clock::to_time_t(high_resolution_clock::now());

		memcpy(&video_string[0], video_image, VIDEO_FRAME_MAX_SIZE);
		frame_message.set_allocated_fvideo_data(&video_string);

		frame_message.set_timestamp(timestamp);

#if defined(USE_POINT_CLOUD) && defined(PROCESS_CLOUD_DISTRIBUTED)
		PCLUtil::convertToXYZPointCloud(frame_message, (uint16_t*) depth_image
			, DEPTH_FRAME_HEIGHT, DEPTH_FRAME_WIDTH);
#else
		memcpy(&depth_string[0], depth_image, DEPTH_FRAME_MAX_SIZE);
		frame_message.set_allocated_fdepth_data(&depth_string);
#endif

		server.sendFrameMessage(frame_message);
	}

	kinect.setLed(LED_BLINK_GREEN);

	return 0;
}
