#!/usr/bin/env python
import rospy
#import numpy as np

from sensor_msgs.msg import JointState

#########################################
class tmotor_driver(object):

    #######################################    
    def __init__(self):

        # Init subscribers 
        self.sub_cmd    = rospy.Subscriber("joints_cmd", JointState , self.cmd_received, queue_size=1)
        
        # Init publishers
        self.pub_sensor = rospy.Publisher("joints_sensor", JointState , queue_size=1)
    
        # Timer
        #self.dt         = 0.02
        #self.timer      = rospy.Timer( rospy.Duration( self.dt ), self.timed_controller )
        
        #################
        # Paramters
        #################
        

        #################
        # Memory
        #################
        
        # cmd
        self.motors_cmd_mode = ['Disable','Disable']
        self.motors_cmd_pos  = [ 0.0 , 0.0 ]
        self.motors_cmd_vel  = [ 0.0 , 0.0 ]
        self.motors_cmd_tor  = [ 0.0 , 0.0 ]
        
        # sensor
        self.motors_names       = ['Joint 0','Joint 1']
        self.motors_sensor_pos  = [ 0.0 , 0.0 ]
        self.motors_sensor_vel  = [ 0.0 , 0.0 ]
        self.motors_sensor_tor  = [ 0.0 , 0.0 ]
        

    ####################################### 
    def cmd_received( self, JointState ):
        """ """
        
        self.motors_cmd_mode = JointState.name # Mode is in the name field
        self.motors_cmd_pos  = JointState.position
        self.motors_cmd_vel  = JointState.velocity
        self.motors_cmd_tor  = JointState.effort
        
        # Main loop is here
        self.send_cmd_to_tmotors()
        
      
    ##########################################################################################
    def send_cmd_to_tmotors(self):
        """ """
        #TODO
        
        # Send commonds to both motor and read sensor data
        
        #Place holder for sensor feedback
        self.motors_sensor_pos  = [ 0.0 , 0.0 ]
        self.motors_sensor_vel  = [ 0.0 , 0.0 ]
        self.motors_sensor_tor  = [ 0.0 , 0.0 ]
        
        self.publish_sensor_data()


    ##########################################################################################
    def publish_sensor_data(self):
        """ """
        #Init msg
        motors_msg = JointState()

        motors_msg.name     = self.motors_names
        motors_msg.position = self.motors_sensor_pos
        motors_msg.velocity = self.motors_sensor_vel
        motors_msg.effort   = self.motors_sensor_tor

        # Publish msg
        self.pub_sensor.publish( motors_msg )
        

#########################################

if __name__ == '__main__':
    
    rospy.init_node('tmotors',anonymous=False)
    node = tmotor_driver()
    rospy.spin()
