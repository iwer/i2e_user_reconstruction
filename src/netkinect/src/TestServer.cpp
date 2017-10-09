#include <iostream>
#include <time.h>
#include <stdint.h>
#include <stdio.h>
#include <string>

#include "Connection.h"
#include "KinectWrapper.h"

#define LOOPS 10

using namespace std;

int main(void){
	size_t frame_buffer_size = KinectWrapper::getBufferSizeForBothFrames();
	char frame_buffer[frame_buffer_size] = {0};
	int client_socket = 0;

	clock_t start_time;
	clock_t end_time;
	clock_t diff_time;
	clock_t avg_time;
	clock_t worst_time;
	clock_t best_time;

	Connection con(CONNECTION_PORT, "127.0.0.1");
	con.createConnection(SERVER);

	con.acceptConnection(&client_socket);

	Connection client(client_socket);

	printf("running server test for %d loops\n", LOOPS);

	avg_time = 0;
	worst_time = 0;
	best_time = 2147483647;
	for(int i=0; i<LOOPS; i++){
		start_time = clock();
		client.recvData(frame_buffer, frame_buffer_size);

		end_time = clock();
		diff_time = end_time - start_time;

		cout << "output: " << string(frame_buffer) << endl;

		if (avg_time == 0){
			avg_time = diff_time;
		} else {
			avg_time = (avg_time + diff_time) / 2;
		}

		if (best_time > diff_time){
			best_time = diff_time;
		}

		if (worst_time < diff_time){
			worst_time = diff_time;
		}
	}


	printf("avg_time = %f milliseconds\n", (double) (avg_time) / 1000);
	printf("best_time = %f milliseconds\n", (double) (best_time) / 1000);
	printf("worst_time = %f milliseconds\n", (double) (worst_time) / 1000);

	con.closeConnection();
	client.closeConnection();

	return 0;
}
