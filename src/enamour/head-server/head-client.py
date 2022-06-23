import select
import socket

import sys

def sendTCP(ip, string):
	ip = ip

	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.connect((ip, 12345))
	sock.setblocking(False)

	sock.sendall(string)

	ready = select.select([sock], [], [], 2)
	data = None

	if ready[0]:
		data = sock.recv(1024)

		sock.close()

		return data

ip = str(sys.argv[1])
roll = int(sys.argv[2])
pitch = int(sys.argv[3])
yaw = int(sys.argv[4])
time = int(sys.argv[5])

string = "angles:" + str(roll) + "," + str(pitch) + "," + str(yaw) + "," + str(time)

print(string)

print(sendTCP(ip, string))









