#ifndef _KINECT_WRAPPER_H_
#define _KINECT_WRAPPER_H_

#include <stdint.h>

extern "C"{
#include "libfreenect/libfreenect_sync.h"
}
#include "../gen/KinectFrameMessage.pb.h"

typedef enum{
    DEPTH,
    VIDEO
} FrameInfo;

/*
typedef enum {
	LED_OFF              = 0,
	LED_GREEN            = 1,
	LED_RED              = 2,
	LED_YELLOW           = 3,
	LED_BLINK_GREEN      = 4,
	LED_BLINK_RED_YELLOW = 6
} freenect_led_options;
*/
typedef freenect_led_options LedOption;


#define USE_POINT_CLOUD

/**
	The size of data for one video frame
*/
#define VIDEO_FRAME_WIDTH 		640
#define VIDEO_FRAME_HEIGHT 		480
//#define VIDEO_FRAME_DEPTH 		CV_8UC3
#define VIDEO_FRAME_MAX_SIZE 	VIDEO_FRAME_HEIGHT * VIDEO_FRAME_WIDTH * 3

/**
	The size of data for one depth frame
*/
#define DEPTH_FRAME_WIDTH 		640
#define DEPTH_FRAME_HEIGHT 		480
//#define DEPTH_FRAME_DEPTH 		CV_16UC1
#define DEPTH_FRAME_MAX_SIZE 	DEPTH_FRAME_HEIGHT * DEPTH_FRAME_WIDTH * 2

class KinectWrapper{
public:
	/**
		Meyers' singleton pattern.
		@return A static reference to the only class instance.
	*/
	static KinectWrapper getInstance();

	/**
		Modified version of https://github.com/PointCloudLibrary/pcl/blob/master/io/src/openni2_grabber.cpp#L594
		to calculate the point cloud without importing the whole library.
	*/
	//static void convertToXYZPointCloud(KinectFrameMessage& message, uint16_t* depth);

	/**
		Wrapping c_syncs freenect_sync_set_led for LED control on Kinect.
		@param op The future state of the LED (see freenect_led_options above).
	*/
	void setLed(LedOption op);

	void handleUSBHandshake();

	/**
		Get a depth or video frame from a Microsoft Kinect using the C-Sync
		Wrapper of the OpenKinect driver.

		@param info The type of frame to get. Either VIDEO or DEPTH.
		@param data Pointer to an uninitialized pointer where the frame data
		will be written.
		@return The return value of freenect_sync_get_{video|depth}_with_res().
	*/
    int getData(FrameInfo info, char** data);

	~KinectWrapper();

private:
	KinectWrapper();

};

#endif
