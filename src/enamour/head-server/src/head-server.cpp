#include <iostream>
#include <cstdlib>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <future>

#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <signal.h>

#include <pigpio.h>

#define PORT 12345
#define BUFSIZE 1024
#define MAX_ANGLE 270
#define MIN_ANGLE -1
#define MAX_PW 2400 // of 2500
#define MIN_PW 600 // from 500
#define MIN_TIME 1000
#define MAX_TIME 10000

typedef struct head_controller_context
{
	bool running;
	int roll_pin, pitch_pin, yaw_pin;
	int roll_angle_prev, pitch_angle_prev, yaw_angle_prev;
	int roll_angle, pitch_angle, yaw_angle;
	int time;
	int tcp_port;
	std::mutex driver_wait;
	std::mutex angle_lock;
} context_st;

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

int map(int x, int in_min, int in_max, int out_min, int out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void set_angle(int gpio, int angle)
{
	//map angle to 500...2500
	gpioServo(gpio, map(angle, MIN_ANGLE, MAX_ANGLE, MIN_PW, MAX_PW));
	//map(angle, MIN_ANGLE, MAX_ANGLE, MIN_PW, MAX_PW);
}

void tcp_server(context_st *context)
{
	int server_fd, conn_fd;
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);
	char buffer[BUFSIZE] = { 0 };

	//puts("Creating socket file descriptor");
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0))== 0) {
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	//puts("Setting socket options");
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(PORT);

	//puts("Binding socket to port 12345");
	if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	while(context->running)
	{
		//puts("Listening for connection attempts");
		if (listen(server_fd, 16) < 0) {
			perror("listen");
			exit(EXIT_FAILURE);
		}
		//puts("Accepting connections");
		if ((conn_fd = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) {
			perror("accept");
			exit(EXIT_FAILURE);
		}
		memset(buffer, 0, sizeof(buffer));
		int n = read(conn_fd, buffer, BUFSIZE);
		if(n == -1)
		{
			perror("read");
			exit(EXIT_FAILURE);
		}

		//printf("read:[%s]\n", buffer);

		if(strstr(buffer, "test") == buffer)
		{
			//puts("received test message");
			int num;
			sscanf(buffer, "test:%d", &num);
			num = num * num;
			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer, "test:%d", num);
		}
		else if(strstr(buffer, "getangles") == buffer)
		{
			//get servo angles and send them
			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer, "angles:%d,%d,%d", context->roll_angle, context->pitch_angle, context->yaw_angle);
		}
		else if(strstr(buffer, "angles") == buffer)
		{
			int ang[3] = { 0 };
			int time;
			sscanf(buffer, "angles:%d,%d,%d,%d", &ang[0], &ang[1], &ang[2], &time);

			//printf("set angles to %d %d %d in %dms\n", ang[0], ang[1], ang[2], time);

			if(context->angle_lock.try_lock())//get exclusive access to angle memory, unless driver running then error.busy
			{
				context->roll_angle = (int)clamp(ang[0], 90, 180);
				context->pitch_angle = (int)clamp(ang[1], 10, 60);
				context->yaw_angle = (int)clamp(ang[2], 90, 180);
				context->time = (int)clamp(time, MIN_TIME, MAX_TIME);

				context->driver_wait.unlock();//unblock driver
				context->angle_lock.unlock();//unacquiring angle memory lock

				memset(buffer, 0, sizeof(buffer));
				sprintf(buffer, "OK");
			}
			else
			{
				memset(buffer, 0, sizeof(buffer));
				sprintf(buffer, "error.busy");
			}
		}
		else
		{
			memset(buffer, 0, sizeof(buffer));
			sprintf(buffer, "error.unexpected");
		}

		send(conn_fd, buffer, strlen(buffer), 0);
		close(conn_fd);
	}
  	// closing the listening socket
	shutdown(server_fd, SHUT_RDWR);
}

void servo_driver(context_st *context)
{
	while(context->running)
	{
		context->driver_wait.lock();//block until unlocked by tcp server
		context->angle_lock.lock();//get exclusive access on angle memory
		for(int i=0; i<context->time && context->running; i+=10)
		{
			if(context->roll_angle > -1)
				set_angle(context->roll_pin,
					context->roll_angle_prev + smootherstep((float)0, (float)context->time, (float)i) * (context->roll_angle - context->roll_angle_prev));

			if(context->pitch_angle > -1)
				set_angle(context->pitch_pin,
					context->pitch_angle_prev + smootherstep((float)0, (float)context->time, (float)i) * (context->pitch_angle - context->pitch_angle_prev));

			if(context->yaw_angle > -1)
				set_angle(context->yaw_pin,
					context->yaw_angle_prev + smootherstep((float)0, (float)context->time, (float)i) * (context->yaw_angle - context->yaw_angle_prev));

			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		if(context->roll_angle > -1)
			context->roll_angle_prev = context->roll_angle;

		if(context->pitch_angle > -1)
			context->pitch_angle_prev = context->pitch_angle;

		if(context->yaw_angle > -1)
			context->yaw_angle_prev = context->yaw_angle;

		context->angle_lock.unlock();
	}
}

int main(int argc, char **argv)
{
	context_st *context = new context_st;

	context->running = true;

	context->roll_pin = 18;
	context->pitch_pin = 23;
	context->yaw_pin = 24;

	context->roll_angle_prev = 135;
	context->pitch_angle_prev = 60;//hard stop at ~90 !
	context->yaw_angle_prev = 135;

	context->roll_angle = 135;
	context->pitch_angle = 60;
	context->yaw_angle = 135;

	context->time = 1000;

	context->tcp_port = 12345;

	gpioInitialise();

	std::thread tcp_server_thread (tcp_server, context);
	std::thread servo_driver_thread (servo_driver, context);

	context->angle_lock.unlock();

	sigset_t w;
	int signo;

	sigemptyset(&w);
	sigaddset(&w, SIGTERM);
	sigwait(&w, &signo);

	context->running = false;

	auto future = std::async(std::launch::async, &std::thread::join, &tcp_server_thread);
	if (future.wait_for(std::chrono::seconds(5)) 
		== std::future_status::timeout)
	{
		pthread_cancel(tcp_server_thread.native_handle());
	}

	future = std::async(std::launch::async, &std::thread::join, &servo_driver_thread);
	if (future.wait_for(std::chrono::seconds(5)) 
		== std::future_status::timeout)
	{
		pthread_cancel(servo_driver_thread.native_handle());
	}

	/*tcp_server_thread.join();
	servo_driver_thread.join();*/

	gpioTerminate();

	return 0;
}
