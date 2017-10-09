#include "UDPConnection.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <string.h>
#include <iostream>
#include "Logger.h"

using namespace std;

#define MAX_UDP_FRAME 1024
#define MIN(x,y) (x < y ? x : y)

UDPConnection::UDPConnection(int port)
	: _port(port)
	, _last_errno(0)
	, _last_sender("") {
	_socket = 0;
	_type = UNDEFINED;
	_info = {};
	_last_sender.resize(INET_ADDRSTRLEN);

	LOG_DEBUG << "created udp connection object with port " << port << endl;
}

void UDPConnection::enableBroadcast(){
   int b = 1;
   setsockopt(_socket, SOL_SOCKET, SO_BROADCAST, &b, sizeof b);
}

void UDPConnection::setRecvTimout(int usec){
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = usec;
	setsockopt(_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

int UDPConnection::createConnection(ConnectionType type, int port, string ip){
	_type = type;
	if (port != -1){
		_port = port;
	}

	_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (_socket < 0){
		_last_errno = errno;
	   LOG_ERROR << "failed to create socket " << strerror(errno) << endl;
	   return -1;
	}

	memset((char *) &_info, 0, sizeof(_info));
	_info.sin_family = AF_INET;
	_info.sin_port = htons(_port);
	_info.sin_addr.s_addr = INADDR_ANY;

	if (bind(_socket, (struct sockaddr*) &_info, sizeof(_info)) != 0){
		_last_errno = errno;
	   LOG_ERROR << "failed to bind socket " << strerror(errno) << endl;
	   closeConnection();
	   return -1;
	}

	LOG_DEBUG << "succesfully created and bound socket " << _socket
	 	<< " port " << _port << endl;

	return 0;
}

int UDPConnection::sendData(const void *buffer, size_t buffer_size, string ip){
	inet_pton(AF_INET, ip.c_str(), (void* )&(_info.sin_addr));

	if (_socket){
		if ( sendto(_socket, (void*) buffer, buffer_size, 0
			, (struct sockaddr *) &_info, sizeof(_info)) < 0){
			_last_errno = errno;
			LOG_ERROR << "failed to send data " << strerror(errno) << endl;
		} else {
			LOG_DEBUG << "sent udp packet, socket: " << _socket << " address: "
				<< ip << " port: " << _port << endl;
			return 0;
		}
	} else {
		LOG_ERROR << "failed to send data because the socket is closed"	<< endl;
	}

	return -1;
}

int UDPConnection::recvData(void* buffer, size_t buffer_size){
	struct sockaddr_in server;
	socklen_t addrin_len = sizeof(server);

	if (_socket){
		if (recvfrom(_socket, (void*) buffer, buffer_size, 0
			, (struct sockaddr *) &server, &addrin_len) < 0){
			if (errno == EAGAIN){
				return 1;
			}

			_last_errno = errno;
			LOG_ERROR << "failed to receive data " << strerror(errno)
				<< endl;
		} else {
			char ip[INET_ADDRSTRLEN];
			inet_ntop(AF_INET, (void *) &(server.sin_addr), ip, INET_ADDRSTRLEN);

			if (strcmp("127.0.0.1", ip) == 0){
				return -1;
			}

			_last_sender.assign(ip, strlen(ip));
			LOG_DEBUG << "received udp packet, socket: " << _socket << " address: "
			 	<< _last_sender << " port: " << ntohs(server.sin_port)
				<< endl;

			return 0;
		}
	} else {
		LOG_ERROR << "failed to send data because the socket is closed"	<< endl;
	}

	return -1;
}
