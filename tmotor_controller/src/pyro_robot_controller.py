#!/usr/bin/env python
import rospy
import numpy as np
import time
import matplotlib
import matplotlib.pyplot as plt

from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

from pyro.control  import robotcontrollers
from pyro.dynamic  import manipulator
from pyro.dynamic  import pendulum
from pyro.control  import nonlinear


#########################################
class robot_controller(object):

    #######################################    
    def __init__(self):

        # Init subscribers  
        self.sub_joy    = rospy.Subscriber("joy", Joy , self.read_joy , queue_size=1) # self.read_joy is the function called when a message is received.
        self.sub_sensor = rospy.Subscriber("joints_sensor", JointState , self.read_joints, queue_size=1) 

        # Init publishers
        self.pub_cmd    = rospy.Publisher("joints_cmd", JointState , queue_size=1)
    
        # Control Timer
        self.dt         = 0.01
        self.timer      = rospy.Timer( rospy.Duration( self.dt ), self.timed_controller )
        
        #################
        # Paramters
        #################
        
        # Robot model
        self.sys         = pendulum.DoublePendulum()
        self.sys.l1      = 0.3
        self.sys.l2      = 0.3
        self.sys.lc1     = 0.3
        self.sys.lc2     = 0.3
        self.sys.I1      = 0.5
        self.sys.I2      = 0.2
        self.sys.m1      = 0.9
        self.sys.m2      = 0.05
        self.sys.u_lb[0] = -1.0
        self.sys.u_lb[0] = -1.0
        self.sys.u_ub[0] = 1.0
        self.sys.u_ub[0] = 1.0
        
        # Computed torque controller
        self.ct_ctl      = nonlinear.ComputedTorqueController( self.sys )
        self.ct_ctl.w0   = 2.0
        self.ct_ctl.zeta = 0.7
        self.ct_ctl.rbar = np.array([0.0,0.0])

        
        # Joint impedance controller
        
        dof = 2
        
        self.joint_pd      = robotcontrollers.JointPD( dof )
        self.joint_pd.kp   = np.array([  3.0, 3.0 ])
        self.joint_pd.kd   = np.array([  1.0,  1.0 ])
        

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
        
        # Robot command
        self.u  = np.array( self.motors_cmd_tor )
        
        # Sensings inputs
        self.x  = np.array([ 0.0, 0.0, 0.0, 0.0]) # State of the system
        self.q  = np.array([ 0.0, 0.0]) # Joint angles of the system
        self.dq = np.array([ 0.0, 0.0]) # Joint velocities of the system
        
        #Joy input
        self.controller_mode = 0
        
        #################
        # Initialization
        #################
        
        # Start loop
        #self.pubish_joints_cmd_msg()
        
        # Start graphic
        self.animator = self.sys.get_animator()
        self.sys.l_domain = 0.6
        #self.animator.show( self.q )
        self.animator.show_plus( self.x , self.u, 0 )
        self.animator.showfig.canvas.draw()
        plt.show(block=False)
        
        # Graphic output Timer
        self.dt2        = 0.1
        self.timer2     = rospy.Timer( rospy.Duration( self.dt2 ), self.timed_graphic )

        
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
                """ velocity control """
                
                self.motors_cmd_vel[0] = self.user_ref[1] * 3.1415 * 2.0
                self.motors_cmd_vel[1] = self.user_ref[0] * 3.1415 * 2.0
                self.motors_cmd_mode   = ['velocity','velocity']
            
            elif ( self.controller_mode == 2 ):
                """ position control """
                
                self.motors_cmd_pos[0] = self.user_ref[1] * 3.1415 * 0.5
                self.motors_cmd_pos[1] = self.user_ref[0] * 3.1415 * 0.25
                self.motors_cmd_mode   = ['position','position']
                
            elif ( self.controller_mode == 3 ):
                """ torque control """
                #print('\n Torque mode')
                
                u = np.array([ self.user_ref[0] * 1.0 , self.user_ref[1] * 1.0   ])
                
                self.motors_cmd_tor[0] = u[1]
                self.motors_cmd_tor[1] = u[0]
                
                self.motors_cmd_mode = ['torque','torque']
                
            elif ( self.controller_mode == 4 ):
                """ RT """
                pass 
                
            elif ( self.controller_mode == 5 ):
                """ computed torque controller """
                #print('\nComputed torque mode')
                
                x  = self.x 
                r  = np.array([3.14-3.1415,0.0])
                t  = 0 #TODO
                u  = self.ct_ctl.c( x, r, t)
                
                print('state:',x)
                print('cmd:',u)
                
                self.motors_cmd_tor[0] = u[1]
                self.motors_cmd_tor[1] = u[0]
                self.motors_cmd_mode = ['torque','torque']
                
            elif ( self.controller_mode == 6 ):
                """ RT """
                pass 
            
            elif ( self.controller_mode == 7 ):
                """ a:  Joint PD """
                #print('\nJoint PD mode')
                
                x  = self.x 
                r  = np.array([0.0-3.14,0.0])
                t  = 0 #TODO
                u  = self.joint_pd.c( x, r, t)
                
                self.motors_cmd_tor[0] = u[1]
                self.motors_cmd_tor[1] = u[0]
                self.motors_cmd_mode = ['torque','torque']
            
            elif ( self.controller_mode == 8 ):
                """ y : gravity compensation"""
                
                x  = self.x 
                r  = np.array([0.0,0.0])
                t  = 0 #TODO
                u  = self.sys.g( self.q )
                
                self.motors_cmd_tor[0] = u[1]
                self.motors_cmd_tor[1] = u[0]
                self.motors_cmd_mode = ['damped_torque','damped_torque']
            
            elif ( self.controller_mode == 9 ):
                """ enable motors and set zero position """

                self.motors_cmd_mode = ['enable','enable']
            
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
            elif(joy_msg.buttons[0]):   
                
                self.controller_mode   = 5
                
            #If button B is active 
            elif(joy_msg.buttons[1]):   
                
                self.controller_mode   = 6
                
            #If button x is active 
            elif(joy_msg.buttons[2]):   
                
                self.controller_mode   = 7
                
            #If button y is active 
            elif(joy_msg.buttons[3]):   
                
                self.controller_mode   = 8
                
            # big button
            elif (joy_msg.buttons[8]):
                
                self.controller_mode   = 9
                
            
            # No active button
            else:
                self.controller_mode   = 1
        
        # Deadman is un-pressed
        else:
            
            self.user_ref        = [ 0.0 , 0.0 ]   
            self.controller_mode = 0 
            
        #self.timed_controller( None )
      
      
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
        
        """
        self.x[0] = msg.position[0]
        self.x[1] = msg.position[1]
        self.x[2] = msg.velocity[0]
        self.x[3] = msg.velocity[1]
        """
        
        self.q  = np.array([ msg.position[1] - 3.1415 , msg.position[0] ])
        self.dq = np.array([ msg.velocity[1] , msg.velocity[0] ])
        
        self.x  = self.sys.q2x( self.q , self.dq )
        self.u  = np.array([ msg.effort[1] , msg.effort[0] ])
        #self.timed_controller( None )
        
        
    #######################################
    def timed_graphic(self, timer):
        
        """
        lines_pts = self.sys.forward_kinematic_lines( self.q )[0]
        robot_line = lines_pts[1]
        self.animator.showlines[1].set_data( robot_line[:, 0 ], robot_line[:, 1 ])
        self.animator.showfig.canvas.draw()
        """
        #print(self.controller_mode)
        self.animator.show_plus_update( self.x, self.u, 0.0 )


#########################################

if __name__ == '__main__':
    
    plt.ion()
    matplotlib.use('Qt4Agg')
    #plt.ioff()
    rospy.init_node('controller',anonymous=False)
    node = robot_controller()
    
    plt.show( block = True )
    rospy.spin()
