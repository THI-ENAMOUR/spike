IP=$1
export ROS_MASTER_URI=http://192.168.123.161:11311/
export ROS_IP=$IP
export ROS_HOSTNAME=$IP

roscore
