#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState


#########################################
class robot_controller(object):

    #######################################    
    def __init__(self):

        # Init subscribers  
        self.sub_joy    = rospy.Subscriber("joy", Joy , self.read_joy , queue_size=1) # self.read_joy is the function called when a message is received.
        self.sub_sensor = rospy.Subscriber("joints_sensor", JointState , self.read_joints, queue_size=1) 

        # Init publishers
        self.pub_cmd    = rospy.Publisher("joints_cmd", JointState , queue_size=1)
    
        # Timer
        self.dt         = 0.02
        self.timer      = rospy.Timer( rospy.Duration( self.dt ), self.timed_controller )
        
        #################
        # Paramters
        #################
        
        # Controller

        #################
        # Memory
        #################
        
        # References Inputs
        self.user_ref        = [ 0.0 , 0.0 ]
        self.controller_mode = 0  # Control mode of this controller node
        
        # Ouput commands
        self.motors_cmd_mode = ['disable','disable']
        self.motors_cmd_pos  = [ 0.0 , 0.0 ]
        self.motors_cmd_vel  = [ 0.0 , 0.0 ]
        self.motors_cmd_tor  = [ 0.0 , 0.0 ]
        
        # Sensings inputs
        self.x = np.array([ 0.0, 0.0, 0.0, 0.0]) # State of the system

        
    #######################################
    def timed_controller(self, timer):
             
        if (self.controller_mode == 0 ):
            
            # Full stop mode
            self.motors_cmd_mode = ['disable','disable']
            self.motors_cmd_pos  = [ 0.0 , 0.0 ]
            self.motors_cmd_vel  = [ 0.0 , 0.0 ]
            self.motors_cmd_tor  = [ 0.0 , 0.0 ]
            
        else:
            ##########################
            # Controllers HERE            
            ##########################
            if  ( self.controller_mode == 1 ):
                """ position control """
                
                self.motors_cmd_vel[0] = self.user_ref[0] * 3.1415 * 2
                self.motors_cmd_vel[1] = self.user_ref[1] * 3.1415 * 2
                
                self.motors_cmd_mode = ['velocity','velocity']
            
            elif ( self.controller_mode == 2 ):
                """ velocity control """
                
                self.motors_cmd_pos[0] = self.user_ref[0] * 3.1415
                self.motors_cmd_pos[1] = self.user_ref[1] * 3.1415
                
                self.motors_cmd_mode = ['position','position']
                
            elif ( self.controller_mode == 3 ):
                """ torque control """
                
                self.motors_cmd_tor[0] = self.user_ref[0] * 1.0
                self.motors_cmd_tor[1] = self.user_ref[1] * 1.0
                
                self.motors_cmd_mode = ['torque','torque']
                
            elif ( self.controller_mode == 4 ):
                pass 
                
            elif ( self.controller_mode == 5 ):
                """ automated mode 2 """
                
                
            elif ( self.controller_mode == 6 ):
                """ enable motors and set zero position """

                self.motors_cmd_mode = ['enable','enable']
                pass
            
            elif ( self.controller_mode == 7 ):
                """ automated mode 3 """
                pass
            
            elif ( self.controller_mode == 8 ):
                """ automated mode 4 """
                pass
            
            elif ( self.controller_mode == 9 ):
                """ automated mode 5 """
                pass
            
        self.pubish_joints_cmd_msg()


    ####################################### 
    def read_joy( self, joy_msg ):
        """ """
    
        self.user_ref        = [ joy_msg.axes[1] , joy_msg.axes[4] ]   # Up-down [left,right] joystick 
        self.controller_mode = 0            
                
        # Software deadman switch
        # If left button is active 
        if (joy_msg.buttons[4]):
            
            #If right button is active       
            if (joy_msg.buttons[5]):   
                
                self.controller_mode   = 2
                
            #If left rigger is pressed
            elif(joy_msg.axes[2] < 0.0):
                
                self.controller_mode   = 3
                
            #If right rigger is pressed
            elif(joy_msg.axes[5] < 0.0):
                
                self.controller_mode   = 4
                
            #If button A is active 
            elif(joy_msg.buttons[2]):   
                
                self.controller_mode   = 5
                
            #If button B is active 
            elif(joy_msg.buttons[8]):   
                
                self.controller_mode   = 6
                
            #If button x is active 
            elif(joy_msg.buttons[0]):   
                
                self.controller_mode   = 7
                
            #If button y is active 
            elif(joy_msg.buttons[3]):   
                
                self.controller_mode   = 8
                
            #If left trigger is active 
            elif (joy_msg.buttons[6]):
                
                self.controller_mode   = 9
                
            
            # No active button
            else:
                self.controller_mode   = 1
        
        # Deadman is un-pressed
        else:
            
            self.user_ref        = [ 0.0 , 0.0 ]   
            self.controller_mode = 0 
      
      
    ##########################################################################################
    def pubish_joints_cmd_msg(self):
 
        #Init msg
        motors_msg = JointState()

        motors_msg.name     = self.motors_cmd_mode
        motors_msg.position = self.motors_cmd_pos
        motors_msg.velocity = self.motors_cmd_vel
        motors_msg.effort   = self.motors_cmd_tor

        # Publish msg
        self.pub_cmd.publish( motors_msg )


    ####################################### 
    def read_joints( self, msg):

        self.x[0] = msg.position[0]
        self.x[1] = msg.position[1]
        self.x[2] = msg.velocity[0]
        self.x[3] = msg.velocity[1]

#########################################

if __name__ == '__main__':
    
    rospy.init_node('controller',anonymous=False)
    node = robot_controller()
    rospy.spin()
