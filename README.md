# Skywriting using Tello Edu

This project is about drawing a letter using a Tello Edu Drone.

## Installation

First, you need to install the [Tello-driver-ros](https://github.com/TIERS/tello-driver-ros) in your catkin workspace (You only need to clone the repository and install the dependencies).

Then, clone this git project into your `catkin_ws/src` and build it:

```
cd ~/catkin_ws
catkin init
catkin_make
```

## Launching the Project

- Turn on the Tello drone.
- Connect to the drone's WiFi access point.
- Open four terminal windows.

In the first terminal, launch the driver:

```
source ~/catkin_ws/devel/setup.bash
roslaunch tello_driver tello_node.launch tello_ip:="192.168.XX.XX"
```

In the second terminal, launch the command to visualize the odometry with TF in Rviz:

```
source ~/catkin_ws/devel/setup.bash
roslaunch odom_to_tf_broadcaster odom_to_tf_broadcaster.launch
```

In the third terminal, run the following command to take off the Drone:

```
rostopic pub /tello/takeoff std_msgs/Empty
```


In the last terminal, run the following and enter the letter you want the drone to draw:

```
source ~/catkin_ws/devel/setup.bash
rosrun tello_twist_control tello_twist_control.py cmd_vel:=/tello/cmd_vel
```

Once the drone has completed drawing the letter, you can run the following command to land it:
```
rostopic pub /tello/land std_msgs/Empty
```