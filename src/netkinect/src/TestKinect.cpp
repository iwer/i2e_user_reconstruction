#include <iostream>
#include <time.h>
#include <stdint.h>
#include <stdio.h>

#include "KinectWrapper.h"

#define LOOPS 1800

using namespace std;

int main(void){
	KinectWrapper kw = KinectWrapper::getInstance();
	char frame_buffer[424688] = {0};

	clock_t start_time;
	clock_t end_time;
	clock_t diff_time;
	clock_t avg_time;
	clock_t worst_time;
	clock_t best_time;

	//kw.getData(DEPTH, frame_buffer);
	kw.getData(VIDEO, frame_buffer);

	printf("running depth and rgb frame get for %d loops\n", LOOPS);

	avg_time = 0;
	worst_time = 0;
	best_time = 2147483647;
	for(int i=0; i<LOOPS; i++){
		start_time = clock();

		//kw.getData(DEPTH, frame_buffer);
		kw.getData(VIDEO, frame_buffer);

		end_time = clock();
		diff_time = end_time - start_time;

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

		clock_t wait_time = clock() + 33333 - diff_time;
		while (wait_time - clock()  > 0){}
	}


	printf("avg_time = %lf milliseconds\n", (double) (avg_time) / 1000);
	printf("best_time = %lf milliseconds\n", (double) (best_time) / 1000);
	printf("worst_time = %lf milliseconds\n", (double) (worst_time) / 1000);


	return 0;
}
