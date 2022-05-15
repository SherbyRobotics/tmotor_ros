# Template for ROS-Tmotor control architecture

1) read joystick buttons to use as a set-point
2) compute command in a python controller script
3) command is send to Tmotor ....
4) have fun with your robot


![Screenshot from 2022-05-15 15-58-10](https://user-images.githubusercontent.com/16725496/168492122-c4571cdc-57b0-472a-a6d9-657b00b193ee.png)


adding to .basrch :

export PYTHONPATH=$PYTHONPATH:"""path-to"""/pyro

source /opt/ros/kinetic/setup.bash
source /home/alex/ros_ws/devel/setup.sh
