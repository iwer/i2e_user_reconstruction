#include "TCPConnection.h"

#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <errno.h>
#include <iostream>
#include <string.h>

#include "Logger.h"

using namespace std;

TCPConnection::TCPConnection(int socket){
	_socket = socket;
	_type = CLIENT;
	_info = {};
	LOG_DEBUG << "created tcp connection object with socket " << socket << endl;
}

int TCPConnection::createConnection(ConnectionType type, int port, string ip){
	if (_type != UNDEFINED){
		return -1;
	}
	_type = type;

	_socket = socket(AF_INET, SOCK_STREAM, 0);
	if (_socket < 0){
		LOG_ERROR << "failed to create socket, strerror : " << strerror(errno)
			<< endl;

		return -1;
	}

	_info.sin_family = AF_INET;
	_info.sin_port = htons(port);

	if (_type == SERVER){
		_info.sin_addr.s_addr = INADDR_ANY;

		int one = 1;
  	if (setsockopt(_socket, SOL_SOCKET, SO_TIMESTAMP, &one, sizeof(one))){
			LOG_ERROR << "failed to set timestamp option, strerror : "
				<< strerror(errno) << endl;

			closeConnection();
			return -1;
		}

		if (bind(_socket, (struct sockaddr*) &_info, sizeof(_info)) != 0){
			LOG_ERROR << "failed to bind the socket, strerror : "
				<< strerror(errno) << endl;

			closeConnection();
			return -1;
		}

		listen(_socket, MAX_CLIENTS);

		LOG_DEBUG << "server socket succesfully created, listening " << _socket
			<< endl;
	} else if (_type == CLIENT){
		inet_pton(AF_INET, ip.c_str(), &_info.sin_addr);

		if (connect(_socket, (struct sockaddr*) &_info, sizeof(_info)) != 0){
			LOG_DEBUG << "failed to connect to server, strerror : "
				<< strerror(errno) << endl;

			closeConnection();
			return -1;
		}

		LOG_DEBUG << "client connection succesfully created " << _socket
			<< endl;
	}

	return 0;
}

int TCPConnection::acceptConnection(){
	if (_type != SERVER){
		LOG_WARNING << "called acceptConnection() with non SERVER type" << endl;
		return -1;
	}

	int socket = 0;
	socklen_t client_size = sizeof(struct sockaddr_in);

	if ((socket = accept(_socket, (struct sockaddr*) &_info
			, &client_size)) < 0){
		if (!(errno == EAGAIN || errno == EWOULDBLOCK)){
			LOG_WARNING << "accepting new client failed, strerror : "
			<< strerror(errno) << endl;
		}

		return -1;
	}

	LOG_DEBUG << "accepted new client on " << _socket << ", new socket "
		<< socket << endl;

	return socket;
}

int TCPConnection::sendData(const void* buffer, size_t buffer_size, std::string ip){
	if (_socket){
		if (send(_socket, buffer, buffer_size, MSG_NOSIGNAL) <= 0){
			LOG_ERROR << "failed to send data (" << _socket << "), strerror : "
			 	<< strerror(errno) << endl;

			closeConnection();
		} else {
			LOG_DEBUG << "sent " << buffer_size << " bytes, socket: "
				<< _socket << endl;
			return 0;
		}
	} else {
		LOG_ERROR << "failed to send data because the socket closed" << endl;
	}
	return -1;
}

int TCPConnection::recvData(void* buffer, size_t buffer_size){
	int ret = 0;

	if (_socket){
		if ((ret = recv(_socket, buffer, buffer_size, MSG_WAITALL)) < 0){
			//ret = _recvChunks(buffer, buffer_size)
			LOG_ERROR << "failed to receive data (" << _socket
				<< "), strerror : " << strerror(errno) << endl;
			closeConnection();
		} else if (ret == 0) {
			LOG_ERROR << "received 0 data, closing connection" << endl;
			closeConnection();
		} else {
			LOG_DEBUG <<"received " << ret << " bytes over socket " << _socket
				<< endl;
			return 0;
		}


	} else {
		LOG_ERROR << "failed to receive data because the socket closed" << endl;
	}
	return -1;
}

/*int TCPConnection::_recvChunks(void* buffer, int buffer_size){
	int ret = 0;
	int recevied_bytes = 0;
	while(recevied_bytes != buffer_size){
		if (_socket){
			if ((ret = recv(_socket, (void*) &((char*)buffer)[recevied_bytes]
					, buffer_size - recevied_bytes, 0)) < 0){
				return ret;
			}
			recevied_bytes += ret;
		} else {
			return 0;
		}
	}
	return recevied_bytes;
}*/

/*int TCPConnection::_getTimestamp(){
	char ctrl[CMSG_SPACE(sizeof(struct timeval))];
    struct cmsghdr *cmsg = (struct cmsghdr *) &ctrl;
	if (cmsg->cmsg_level == SOL_SOCKET &&
	   cmsg->cmsg_type  == SCM_TIMESTAMP &&
	   cmsg->cmsg_len   == CMSG_LEN(sizeof(time_kernel)))
   {
	   memcpy(&time_kernel, CMSG_DATA(cmsg), sizeof(time_kernel));
   }
}*/
