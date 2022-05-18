# ROS driver for controlling 2 tmotors using CAN interface

This drive was tested using [PEAK System's PCAN-USB](https://www.peak-system.com/PCAN-USB.199.0.html?&L=1) on ubuntu 20.04. Drivers should be installed by default on ubuntu.
Use `ip link show` to verify that the CAN interface is detected

To communicate with the motors, it is important to set up the CAN interface to 1M baudrate:

	sudo ip link set can0 type can bitrate 1000000

	sudo ip link set up can0

## Dependencies:

**bitstring :**

	pip3 install bitstring

**mini-cheetah-tmotor-python-can :**

	pip3 install mini-cheetah-motor-driver-socketcan
	
