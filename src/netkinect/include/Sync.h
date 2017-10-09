#ifndef _SYNC_H_
#define _SYNC_H_

#include <thread>
#include <google/protobuf/message_lite.h>

#include "Common.h"

#include "UDPConnection.h"

using namespace std;
using namespace google::protobuf;

class Sync{
public:
	Sync();

	int connect();
	void getTime(uint64_t* t);
	int isActive(){ return _running; };

	~Sync();

private:
	UDPConnection _udp_con;

	volatile bool _running;
	thread _sync_thread;
	bool _is_leader;

	void _setTime(int64_t offset_sec, int64_t offset_nsec);
	void _sendMessage(MessageLite& m, string ip);
	int _recvMessage(MessageLite& m);
	void _threadHandle();

	void __berkleyAlgorithm();
};

#endif
