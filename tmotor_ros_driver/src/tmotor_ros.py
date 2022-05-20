#!/usr/bin/env python
import rospy
from motor_driver.canmotorlib import CanMotorController
import numpy as np

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
        
        self.offline_debug = False

        #################
        # Motors init
        #################

        self.tmotors = [CanMotorController(can_socket='can0', motor_id=0x01, socket_timeout=0.5), CanMotorController(can_socket='can0', motor_id=0x02, socket_timeout=0.5)]
        self.tmotors[0].change_motor_constants(-12.5, 12.5, -41.0, 41.0, 0, 500, 0, 50, -9.0, 9.0)
        self.tmotors[1].change_motor_constants(-12.5, 12.5, -41.0, 41.0, 0, 500, 0, 50, -9.0, 9.0)
        self.tmotors_params = [ {'kp': 20, 'kd': 5} , {'kp': 20, 'kd': 5} ]
        

        #################
        # Memory
        #################
        
        # cmd for tmotors
        self.motors_cmd_mode = ['disable','disable']
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
        self.motors_cmd_vel  = list(JointState.velocity)
        self.motors_cmd_tor  = list(JointState.effort)
        
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
            
            # axis limit for motor 1 : [-1 turn, 1 turn]
            if self.motors_sensor_pos[1] > 6.5 and self.motors_cmd_vel[1] > 0:
                self.motors_cmd_vel[1] = 0.0
            if self.motors_sensor_pos[1] < -6.5 and self.motors_cmd_vel[1] < 0:
                self.motors_cmd_vel[1] = 0.0

            if self.motors_sensor_pos[1] > 6.5 and self.motors_cmd_tor[1] > 0:
                self.motors_cmd_tor[1] = 0.0
            if self.motors_sensor_pos[1] < -6.5 and self.motors_cmd_tor[1] < 0:
                self.motors_cmd_tor[1] = 0.0
            
            # Send commonds to both motor and read sensor data
            for i in range(2):
                
                #################################################
                if self.motors_cmd_mode[i] == 'disable':

                    self.tmotors[i].disable_motor()

                #################################################
                elif self.motors_cmd_mode[i] == 'enable':

                    self.tmotors[i].enable_motor()
                    self.tmotors[i].set_zero_position()

                #################################################
                elif self.motors_cmd_mode[i] == 'position':

                    self.motors_sensor_pos[i] , self.motors_sensor_vel[i], self.motors_sensor_tor[i] = self.tmotors[i].send_rad_command(self.motors_cmd_pos[i], 0, self.tmotors_params[i]['kp'], self.tmotors_params[i]['kd'], 0)
                    
                #################################################  
                elif self.motors_cmd_mode[i] == 'velocity':
                    
                    self.motors_sensor_pos[i] , self.motors_sensor_vel[i], self.motors_sensor_tor[i] = self.tmotors[i].send_rad_command(0, self.motors_cmd_vel[i], 0, self.tmotors_params[i]['kd'], 0)
                    
                #################################################   
                elif self.motors_cmd_mode[i] == 'torque':
                    
                    self.motors_sensor_pos[i] , self.motors_sensor_vel[i], self.motors_sensor_tor[i] = self.tmotors[i].send_rad_command(0, 0, 0, 0, self.motors_cmd_tor[i])
                    
        
        
        
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
