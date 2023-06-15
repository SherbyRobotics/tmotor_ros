# ROS-Tmotor control architecture for custom robotic arms

Warning: Experimental code !!

This package is 3 components:

## T-motor Node
A wrapper for using this library https://github.com/dfki-ric-underactuated-lab/mini-cheetah-tmotor-python-can as a ROS node.

## Controller Node 
1) read joystick buttons to use as a set-point and buttons for control mode
2) read motor postion 
3) implement custom control laws: gravity conmpensation, impedance control, computed torque, velocity control, etc.
4) send torque command to the motors

## Architecture
Launch files for starting this control architecture:

![Screenshot from 2022-05-15 15-58-10](https://user-images.githubusercontent.com/16725496/168492122-c4571cdc-57b0-472a-a6d9-657b00b193ee.png)

# Demo

A 2 DoF arm using this code is demonstrated in this video:
https://youtu.be/Cbtk6y84C6I



## Motor initialization

Use `ip link show` to verify that the CAN interface is detected

To communicate with the motors, it is important to set up the CAN interface to 1M baudrate:

	sudo ip link set can0 type can bitrate 1000000

	sudo ip link set up can0

## Dependencies:

**bitstring :**

	pip3 install bitstring

**mini-cheetah-tmotor-python-can :**
  
  NOTE: There is a unsolve bug in the latest version, use our fork for older compatible version
  
**pyro (optional for advanced control mode) :**
  
  
## Installation

adding to .basrch :

export PYTHONPATH=$PYTHONPATH:"""path-to"""/pyro
source /opt/ros/kinetic/setup.bash
source """path-to"""/ros_ws/devel/setup.sh
