#ifndef _CLIENT_H_
#define _CLIENT_H_

#include <arpa/inet.h>
#include <thread>
#include <mutex>

#include "Common.h"

#include "TCPConnection.h"

#include "../gen/KinectFrameMessage.pb.h"

#include "recon/typedefs.h"

using namespace std;

class Client{
public:
	static int leader_id;

	Client(int id, int tcp_socket);

	void setInfo(struct sockaddr_in* info);
	int getVideo(char** video, int size);
	int getDepth(char** depth, int size);
	int getCloud(float** cloud, int size);
	int getCloud(float** cloud, int size, recon::CloudPtr pc);

	void processedData() { _data_available = 0; };

	int isDataAvailable() { return _data_available; };
	int isActive(){ return _running; };

	~Client();


private:
	void _threadHandle();
	void _handleFrameMessage();
	void _recvConnectionMessage();

	TCPConnection _tcp_con;

	KinectFrameMessage _sensor_data;
	volatile int _data_available;

	volatile int _running;
	mutex _data_mutex;
	thread _client_thread;

	bool _use_point_cloud;
	int _video_height;
	int _video_width;
	int _depth_height;
	int _depth_width;
	int _message_size;

	char* _recv_buf;
	int _id;
};

#endif
