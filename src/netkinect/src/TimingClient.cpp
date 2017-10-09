#include "TimingClient.h"

#include <time.h>
#include "../gen/TimingMessage.pb.h"
#include <google/protobuf/message_lite.h>

using namespace google::protobuf;

TimingClient::TimingClient(UDPConnection* con)
	: _con(con)
	, _running(1)
	, _thread(&TimingClient::_threadHandle, this){

}

TimingClient::~TimingClient(){
	_running = 0;
}

void TimingClient::getTime(uint64_t* t){
	struct timeval tv;

	gettimeofday(&tv, NULL);

	t[0] = (uint64_t) tv.tv_sec;
	t[1] = (uint64_t) tv.tv_usec;
}

void TimingClient::setTime(int64_t offset_sec, int64_t offset_usec){
	struct timespec t;
	if(clock_gettime(CLOCK_REALTIME, &t) == -1){
		cout << "settime error: " << strerror(errno) << endl;
	}

	t.tv_sec += offset_sec;
	t.tv_nsec += offset_usec;

	if(clock_settime(CLOCK_REALTIME, &t) == -1){
		cout << "settime error: " << strerror(errno) << endl;
	}
}

void sendMessage2(MessageLite& m, Connection& con){
	int size = m.ByteSize();
	char buffer[255] = {0};

	m.SerializeToArray(&buffer[1], 254);
	buffer[0] = size;

	con.sendData(buffer, 255);
}

void recvMessage2(MessageLite& m, Connection& con){
	char buffer[255] = {0};
	con.recvData(buffer, 255);

	int msg_len = buffer[0];

	m.ParseFromArray(&buffer[1], msg_len);
}

void TimingClient::_threadHandle(){
	uint64_t var[2] = {0};
	TimingMessage tm;

	while (_running){
		if (_con->isClosed()){
			_running = 0;
			break;
		}

		// recv trigger message
		recvMessage2(tm, *_con);

		// send getTime()
		getTime(var);
		tm.set_sec(var[0]);
		tm.set_usec(var[1]);
		cout << "my time = {" << tm.sec() << ", " << tm.usec() << "}" << endl;
		sendMessage2(tm, *_con);

		// recv offset
		recvMessage2(tm, *_con);

		// call setTime(offset)
		cout << "offset = {" << tm.sec() << ", " << tm.usec() << "}" << endl;
		setTime(tm.sec(), tm.usec());
	}

	LOG_DEBUG << "leaving TimingClient::_threadHandle" << endl;
}
