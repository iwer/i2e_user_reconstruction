#ifndef _TIMING_CLIENT_H_
#define _TIMING_CLIENT_H_

#include "Common.h"
#include "UDPConnection.h"
#include <thread>

class TimingClient{
public:
	TimingClient(UDPConnection* con);
	~TimingClient();

	void getTime(uint64_t* t);
	void setTime(int64_t offset_sec, int64_t offset_usec);


private:
	void _threadHandle();

	UDPConnection* _con;
	volatile int _running;
	thread _thread;
};

#endif
