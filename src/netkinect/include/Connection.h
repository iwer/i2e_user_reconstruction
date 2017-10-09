#ifndef _CONNECTION_H_
#define _CONNECTION_H_

#include <fcntl.h>
#include <cstring>
#include <string>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/time.h>

#include "Common.h"
#include "Logger.h"

typedef enum{
	UNDEFINED,
	SERVER,
	CLIENT
} ConnectionType;

class Connection{
public:
	static int next_port;

public:
	/**
		Constructor to create new connection.
	*/
	Connection()
		: _socket(0)
		, _type(UNDEFINED)
		, _info({}) {}

	/**
		Connect to ip_address or listen() on socket. On connect(), listen() or
		bind() failure it logs the errno output string.
		@param type Whether to handle this instance as a server or client.
		@param port The port this connection should use.
		@param ip_address IPv4 address where the instance should connect itself
		to. Can be NULL for type == SERVER.
		@return 0 on success, -1 otherwise.
	*/
	virtual int createConnection(ConnectionType type, int port
		, std::string ip) = 0;

	/**
		Send data over _socket. On send() failure it logs the errno output
		string.
		@param buffer Pointer to the data buffer, which should be sent.
		@param buffer_size The amount of data to be sent from buffer.
	*/
	virtual int sendData(const void* buffer, size_t buffer_size, std::string ip) = 0;

	//Header peekHeader();

	/**
		Receive data from socket. Is blocking. Calls _recvChunks() to receive
		bigger amounts of data. On recv() failure it logs the errno output
		string.
		@param buffer The buffer in which the received data will be stored.
		@param buffer_size The maximum amount of bytes to be written in the
		buffer.
	*/
	virtual int recvData(void* buffer, size_t buffer_size) = 0;

	/**
		Check wether the socket is closed or not.
		@return 1 if the socket is closed, 0 otherwise
	*/
	int isClosed(){
		return _socket==0;
	};

	/**
		Marks the socket as non-blocking.
	*/
	void setNonBlocking(){
		fcntl(_socket, F_SETFL, fcntl(_socket, F_GETFL, 0) | O_NONBLOCK);
	};

	/**
		Copies the info about the connection.
		@param info The struct, from which the connection will copy.
	*/
	void setInfo(struct sockaddr_in* info){
		std::memcpy(&_info, info, sizeof(struct sockaddr_in));
	};

	/**
		Returns the pointer to the struct containig info about the peer.
		@return Pointer to the info struct.
	*/
	struct sockaddr_in* getInfo(){
		return &_info;
	};

	/**
		System call to get a timeval struct containing the arrival of the last
		packet.
		@param tv Pointer to the timeval struct in which the info will be stored
	*/
	void getArrivalOfLastPacket(struct timeval *tv){
		ioctl(_socket, SIOCGSTAMP, tv);
	};

	/**
		Close the socket. The instance will not be deleted, so be carefull
		calling other functions with a closed socket.
	*/
	void closeConnection(){
		if (_socket){
			LOG_DEBUG << "closing socket " << _socket << endl;
			close(_socket);
			_socket = 0;
		}
	};

	virtual ~Connection(){
		if (_socket){
			LOG_WARNING << "connection instance died without closing first"
				<< endl;
			close(_socket);
		}
	}

protected:
	int _socket;				//The socket of the connection.
	ConnectionType _type;		//The type of connection (SERVER or CLIENT).
	struct sockaddr_in _info;	//Contains information about the other part of
								//the connection.

};

#endif
