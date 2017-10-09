#ifndef _UDP_CONNECTION_H_
#define _UDP_CONNECTION_H_

#include <string>

#include "Connection.h"

class UDPConnection: public Connection{
public:

	UDPConnection() : Connection(){};
	UDPConnection(int port);

	int getPort(){ return _port; };
	void setInfo(struct sockaddr_in* info){
		std::memcpy(&_info, info, sizeof(struct sockaddr_in));
		if (_port != 0){
			_info.sin_port = htons(_port);
		}
	};

	void enableBroadcast();
	void setRecvTimout(int usec);
	int getLastErrno() { return _last_errno; };
	string getIPFromLastSender() { return _last_sender; };

	/**
		Inherited from Connection.
	*/
	int createConnection(ConnectionType type, int port, std::string ip);
	int sendData(const void* buffer, size_t buffer_size, std::string ip);
	int recvData(void* buffer, size_t buffer_size);

private:
	int _port;
	int _last_errno;
	string _last_sender;
};

#endif
