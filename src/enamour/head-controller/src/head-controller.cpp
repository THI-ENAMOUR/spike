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

#define PORT 12345
#define BUFSIZE 1024
#define MAX_ANGLE 180
#define MIN_ANGLE 0
#define MAX_PW 2500
#define MIN_PW 500

typedef struct head_controller_context
{
	bool running;
	int gpio_pins[3];
	int angles[3];
	int tcp_port;
	std::mutex mutex;
} context_st;

#include "ros/ros.h"
#include "std_msgs/String.h"

void callback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("received ROS message: [%s]", msg->data.c_str());
}

void tcp_server(context_st *context)
{
	int server_fd, conn_fd;
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);
	char buffer[BUFSIZE] = { 0 };

	puts("Creating socket file descriptor");
	if ((server_fd = socket(AF_INET, SOCK_STREAM, 0))== 0) {
		perror("socket failed");
		exit(EXIT_FAILURE);
	}

	puts("Setting socket options");
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
		perror("setsockopt");
		exit(EXIT_FAILURE);
	}
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = INADDR_ANY;
	address.sin_port = htons(PORT);

	puts("Binding socket to port 12345");
	if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
		perror("bind failed");
		exit(EXIT_FAILURE);
	}
	while(context->running)
	{
		puts("Listening for connection attempts");
		if (listen(server_fd, 16) < 0) {
			perror("listen");
			exit(EXIT_FAILURE);
		}
		puts("Accepting connections");
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

		printf("read:[%s]\n", buffer);

		if(strstr(buffer, "test") == buffer)
		{
			puts("received test message");
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
			sprintf(buffer, "angles:%d,%d,%d", context->angles[0], context->angles[1], context->angles[2]);
		}
		else if(strstr(buffer, "angles") == buffer)
		{
			int ang[3] = { 0 };
			sscanf(buffer, "angles:%d,%d,%d", &ang[0], &ang[1], &ang[2]);

			printf("set angles to %d %d %d\n", ang[0], ang[1], ang[2]);

			if(MIN_ANGLE <= ang[0] && ang[0] <= MAX_ANGLE &&
				MIN_ANGLE <= ang[1] && ang[1] <= MAX_ANGLE &&
				MIN_ANGLE <= ang[2] && ang[2] <= MAX_ANGLE)
			{
				context->mutex.lock();

				context->angles[0] = ang[0];
				context->angles[1] = ang[1];
				context->angles[2] = ang[2];

				context->mutex.unlock();

				memset(buffer, 0, sizeof(buffer));
				sprintf(buffer, "OK");
				puts(buffer);
			}
			else
			{
				memset(buffer, 0, sizeof(buffer));
				sprintf(buffer, "error.range");
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

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void set_angle(int gpio, int angle)
{
	//gpioServo(gpio, map(angle, MIN_ANGLE, MAX_ANGLE, MIN_PW, MAX_PW))
	map(angle, MIN_ANGLE, MAX_ANGLE, MIN_PW, MAX_PW);
}

void servo_driver(context_st *context)
{
	//gpioInitialise();
	while(context->running)
	{
		context->mutex.lock();
		
		for(int i=0; i<3; i++)
		{
			set_angle(context->gpio_pins[i], context->angles[i]);
		}

		context->mutex.unlock();

		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	/*gpioServo(23, 2500);
	getc(stdin);
	gpioTerminate();*/
}

int main(int argc, char **argv)
{
	context_st *context = new context_st;
	context->running = true;
	// TODO
	// gpio pin numbers
	// context->gpio_pins[0,1,2]
	context->angles[0] = 0;
	context->angles[1] = 0;
	context->angles[2] = 0;
	context->tcp_port = 12345;

	/*std::thread tcp_server_thread (tcp_server, context);
	std::thread servo_driver_thread (servo_driver, context);

	tcp_server_thread.join();
	servo_driver_thread.join();*/

	ros::init(argc, argv, "head_controller");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("servo_angles", 1000, callback);

	ros::spin();

	return 0;
}