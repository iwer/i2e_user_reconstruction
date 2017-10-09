#ifndef _SERVER_H_
#define _SERVER_H_

#include <thread>

#include "Common.h"

#include "TCPConnection.h"
#include "../gen/KinectFrameMessage.pb.h"

using namespace std;

class Server{
public:
	Server();

	int connect(int is_leader);
	void sendFrameMessage(KinectFrameMessage& kfm);
	int isClosed(){ return _tcp_con.isClosed(); };

	~Server();


private:
	TCPConnection _tcp_con;
};

#endif
