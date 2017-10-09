#include <iostream>
#include <time.h>
#include <unistd.h>
#include <signal.h>

using namespace std;

#include "Common.h"
#include "Connection.h"
#include "UDPConnection.h"
#include "TimingClient.h"
#include "../gen/TimingMessage.pb.h"
#include <google/protobuf/message_lite.h>

using namespace google::protobuf;

volatile bool running = true;
volatile int us = 0;
volatile int ms = 0;
volatile int sec = 0;

void signalHandler(int signal){
	if (signal == SIGINT
	 || signal == SIGTERM
	 || signal == SIGQUIT){
		running = false;
	}
}

void getTime(uint64_t* t){
	struct timeval tv;

	gettimeofday(&tv, NULL);

	t[0] = (uint64_t) tv.tv_sec;
	t[1] = (uint64_t) tv.tv_usec;
}

void sendMessage(MessageLite& m, Connection& con){
	int size = m.ByteSize();
	char buffer[255] = {0};

	m.SerializeToArray(&buffer[1], 254);
	buffer[0] = size;

	con.sendData(buffer, 255);
}

void recvMessage(MessageLite& m, Connection& con){
	char buffer[255] = {0};
	con.recvData(buffer, 255);

	int msg_len = buffer[0];

	m.ParseFromArray(&buffer[1], msg_len);
}

int main(){
	signal(SIGINT, signalHandler);
	signal(SIGTERM, signalHandler);
	signal(SIGQUIT, signalHandler);

	UDPConnection udp_con(CONNECTION_PORT);

	struct sockaddr_in me;

	memset((char *) &me, 0, sizeof(me));
	me.sin_family = AF_INET;
	me.sin_port = htons(CONNECTION_PORT);

#define SENDER
#ifndef SENDER
	inet_aton("192.168.1.2", &me.sin_addr);
	cout << "CLIENT" << endl;
#else
	inet_aton("192.168.1.234", &me.sin_addr);
	cout << "SERVER" << endl;
#endif

	udp_con.setInfo(&me);
	udp_con.createConnection(CLIENT, -1, "");
#ifndef SENDER
	TimingClient tc(&udp_con);
#else
	TimingMessage tm;
	tm.set_sec(1503930178);
	tm.set_usec(143286);
#endif

	while(running){
#ifdef SENDER
		uint64_t t1[2] = {0};
		uint64_t t2[2] = {0};
		uint64_t rtt[2] = {0};

		sendMessage(tm, udp_con);

		getTime(t1);
		recvMessage(tm, udp_con);
		getTime(t2);
		rtt[0] = (t2[0] - t1[0]) / 2;
		rtt[1] = (t2[1] - t1[1]) / 2;
		cout << "my t = {" << t2[0] << ", " << t2[1] << "} his time = {" << tm.sec() << ", " << tm.usec() << "}" << endl;
		tm.set_sec((t2[0] - tm.sec()) - rtt[0]);
		tm.set_usec((t2[1] - tm.usec()) - rtt[1]);
		sendMessage(tm, udp_con);
		usleep(1000);
#endif
	}

	cout << "ende" << endl;

	return 0;
}
