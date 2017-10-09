#include "Sync.h"

#include <iostream>
#include <unistd.h>
#include <errno.h>

#include "../gen/SyncMessage.pb.h"
#include "Logger.h"

Sync::Sync()
	: _udp_con(CONNECTION_PORT)
	, _running(false)
	, _sync_thread(&Sync::_threadHandle, this)
	, _is_leader(false) {}

Sync::~Sync(){
	_running = false;
	_udp_con.closeConnection();
	_sync_thread.join();
}

int Sync::connect(){
	SyncMessage sm;
	int timeout = 10000;
	int found_leader = 0;

	_udp_con.createConnection(CLIENT, -1, "");
	_udp_con.enableBroadcast();
	_udp_con.setRecvTimout(timeout);

	sm.set_type(SyncMessage_Type_ELECTION);
	sm.set_leader(false);

	_sendMessage(sm, BROADCAST_IP);

	int ret = 0;
	while (1){
		if ((ret = _recvMessage(sm)) == 1){
			//timout
			LOG_DEBUG << "Election timeout" << endl;
			if (found_leader == 0){
				_is_leader = true;
				LOG_DEBUG << "I am the new leader" << endl;
			}
			break;
		} else if (ret == 0) {
			// TODO save ips in list
			LOG_DEBUG << "Received election message from "
				<< _udp_con.getIPFromLastSender() << ", Leader = "
				<< sm.leader() << endl;
			if (sm.type() == SyncMessage_Type_ELECTION && sm.leader()){
				found_leader = 1;
			}
		}
	}

	_running = true;

	return found_leader;
}

void Sync::getTime(uint64_t* t){
	struct timespec tv;

	clock_gettime(CLOCK_REALTIME, &tv);

	t[0] = (uint64_t) tv.tv_sec;
	t[1] = (uint64_t) tv.tv_nsec;
}

void Sync::_setTime(int64_t offset_sec, int64_t offset_nsec){
	struct timespec t;
	if(clock_gettime(CLOCK_REALTIME, &t) == -1){
		cout << "settime error: " << strerror(errno) << endl;
	}

	t.tv_sec += offset_sec;
	t.tv_nsec += offset_nsec;

	if(clock_settime(CLOCK_REALTIME, &t) == -1){
		cout << "settime error: " << strerror(errno) << endl;
	}
}

void Sync::_sendMessage(MessageLite& m, string ip){
	if (m.ByteSize() > 254){
		LOG_WARNING << "serialized SyncMessage length exceeded 254 Bytes ("
			<< m.ByteSize() << ")" << endl;
		return;
	}

	char size = m.ByteSize();
	char buffer[255] = {0};

	m.SerializeToArray(&buffer[1], 254);
	buffer[0] = size;

	_udp_con.sendData(buffer, 255, ip);
}

int Sync::_recvMessage(MessageLite& m){
	char buffer[255] = {0};

	int ret = 0;
	if ((ret = _udp_con.recvData(buffer, 255)) != 0){
		return ret;
	}

	int msg_len = buffer[0];

	m.ParseFromArray(&buffer[1], msg_len);

	return 0;
}

void Sync::__berkleyAlgorithm(){
	if (_is_leader){
		//send broadcast

		// wait for responses

		//average all clock times

		//send offset
	} else {
		// wait for broadcast

		//send time

		//wait for offset

		// adjust time
	}
}

void Sync::_threadHandle(){
	SyncMessage sm;

	//wait until instance is connected
	while (!_running);

	while (_running){
		// TODO implement message queue from sync to server

		if (_recvMessage(sm) != 0){
			continue;
		}

		if (sm.type() == SyncMessage_Type_SYNC){

		} else if (sm.type() == SyncMessage_Type_ELECTION) {
			sm.set_type(SyncMessage_Type_ELECTION);
			sm.set_leader(_is_leader);
			_sendMessage(sm, _udp_con.getIPFromLastSender());
		} else {
			LOG_WARNING << "Unknown SyncMessage type " << sm.type() << endl;
		}

		//__berkleyAlgorithm();
	}

	LOG_DEBUG << "leaving _threadHandle" << endl;
}
