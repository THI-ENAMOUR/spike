# ENAMOUR-Ros-To-A1
This package establishes the communication between the ROS nodes of the Champ framework and the Unitree A1 robot. The node converts ROS messages to the Unitree specific LCM message format and otherwise. The messages are transmitted to the A1 via an LCM server using UDP. The LCM server is also included in this package. Details can be found in the architectural description. [Add Link]
## Build your workspace:

```
cd <your_ws>
catkin_make
source <your_ws>/devel/setup.bash
```
## Setup the net connection
[from the unitree manuel:](https://github.com/unitreerobotics/unitree_ros_to_real)

First, please connect the network cable between your PC and robot. Then run `ifconfig` in a terminal, you will find your port name. For example, `enx000ec6612921`.

Then, open the `ipconfig.sh` , modify the port name to your own. And run the following commands:
```
sudo chmod +x ipconfig.sh
sudo ./ipconfig.sh
```
If you run the `ifconfig` again, you will find that port has `inet` and `netmask` now.
In order to set your port automatically, you can modify `interfaces`:
```
sudo gedit /etc/network/interfaces
```
And add the following 4 lines at the end:
```
auto enx000ec6612921
iface enx000ec6612921 inet static
address 192.168.123.162
netmask 255.255.255.0
```
Where the port name have to be changed to your own.

## Run the setup 

 1. Start the Champ Framework for Unitree A1:
```
roslaunch a1_config bringup.launch rviz:=true
```

 2. Start the LCM-Server:
```
rosrun ros_to_a1 lcm_server_3_2
```

 3. Start the "ros-to-a1" node for converting messages between the Champ framework and Unitree A1 robot:
```
 rosrun ros_to_a1 ros_to_a1_node
 ```