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
        
        self.offline_debug = True
        

        #################
        # Memory
        #################
        
        # cmd for tmotors
        self.motors_cmd_mode = ['Disable','Disable']
        self.motors_cmd_pos  = [ 0.0 , 0.0 ]
        self.motors_cmd_vel  = [ 0.0 , 0.0 ]
        self.motors_cmd_tor  = [ 0.0 , 0.0 ]
        
        # sensor feedback
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
        
        if self.offline_debug:
            
            # Kinematic model for debug
            dt = 0.02
            
            for i in range(2):
                
                #################################################
                if self.motors_cmd_mode[i] == 'position':
                    
                    self.motors_sensor_pos[i] = self.motors_cmd_pos[i]
                    
                #################################################  
                elif self.motors_cmd_mode[i] == 'velocity':
                    
                    self.motors_sensor_vel[i] = self.motors_cmd_vel[i]
                    self.motors_sensor_pos[i] = self.motors_sensor_pos[i] + self.motors_cmd_vel[i] * dt
                    
                #################################################   
                elif self.motors_cmd_mode[i] == 'torque':
                    
                    self.motors_sensor_tor[i] = self.motors_cmd_tor[i]
                    self.motors_sensor_vel[i] = self.motors_sensor_vel[i] + self.motors_cmd_tor[i] * dt
                    self.motors_sensor_pos[i] = self.motors_sensor_pos[i] + self.motors_sensor_vel[i] * dt
                    
        
        else:
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
