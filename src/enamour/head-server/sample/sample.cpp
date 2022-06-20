#include <iostream>
#include <cstdlib>
#include <string>
#include <memory>
#include <thread>
#include <mutex>

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

//#include <pigpio.h>

#define MAX_ANGLE 180
#define MIN_ANGLE 0
#define MAX_PW 2500
#define MIN_PW 500
#define MIN_TIME 0
#define MAX_TIME 10000

float clamp(float x, float lowerlimit, float upperlimit)
{
	if (x < lowerlimit)
		x = lowerlimit;
	if (x > upperlimit)
		x = upperlimit;
	return x;
}

float smootherstep(float edge0, float edge1, float x)
{
	x = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
	return x * x * x * (x * (x * 6 - 15) + 10);
}

float smoothstep (float edge0, float edge1, float x)
{
	if (x < edge0)
		return 0;
	if (x >= edge1)
		return 1;
	x = (x - edge0) / (edge1 - edge0);
	return x * x * (3 - 2 * x);
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void set_angle(int gpio, long angle)
{
	//map 0..180 to 500..2500
	//gpioServo(gpio, map(angle, MIN_ANGLE, MAX_ANGLE, MIN_PW, MAX_PW));
	printf("-");
}

int main(int argc, char **argv)
{
	//gpioInitialise();
	if(argc < 5)
	{
		perror("too few arguments");
		exit(EXIT_FAILURE);
	}

	int gpio = atoi(argv[2]);
	int angle = atoi(argv[3]);
	int time = atoi(argv[4]);

	/*if(strncmp(argv[1], "linear", 6) == 0)
	{
		for(int i=0; i<time; i+=50)
		{

		}
	}
	else */if(strcmp(argv[1], "smooth") == 0)
	{
		for(int i=0; i<time; i+=10)
		{
			int ang = smoothstep((float)0, (float)time, (float)i) * angle;
			set_angle(gpio, ang);
			for(int j=0; j<ang; j+=1)
			{
				set_angle(gpio, ang);
			}
			printf("\n");
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
	else if(strcmp(argv[1], "smoother") == 0)
	{
		for(int i=0; i<time; i+=10)
		{
			int ang = smootherstep((float)0, (float)time, (float)i) * angle;
			set_angle(gpio, ang);
			for(int j=0; j<ang; j+=1)
			{
				set_angle(gpio, ang);
			}
			printf("\n");
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
	else
	{
		perror("error");
		exit(EXIT_FAILURE);
	}

	exit(EXIT_SUCCESS);
}
