#!/usr/bin/env python
import rospy
import numpy as np
import time
import matplotlib
import matplotlib.pyplot as plt
from threading import Thread

from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from std_msgs.msg    import Header

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
        #self.dt         = 0.005
        #self.timer      = rospy.Timer( rospy.Duration( self.dt ), self.timed_controller )
        
        
        #################
        # Paramters
        #################
        
        # Robot model
        #self.sys         = pendulum.DoublePendulum()
        self.sys         = manipulator.TwoLinkManipulator()
        self.sys.l1      = 0.4
        self.sys.l2      = 0.3
        self.sys.lc1     = 0.4
        self.sys.lc2     = 0.3
        self.sys.I1      = 0.05
        self.sys.I2      = 0.05
        self.sys.m1      = 0.6
        self.sys.m2      = 0.03
        self.sys.d1      = 0.0
        self.sys.d2      = 0.0
        self.sys.u_lb[0] = -1.0
        self.sys.u_lb[1] = -1.0
        self.sys.u_ub[0] = 1.0
        self.sys.u_ub[1] = 1.0
        
        # Computed torque controller
        self.ct_ctl      = nonlinear.ComputedTorqueController( self.sys )
        self.ct_ctl.w0   = 2.7
        self.ct_ctl.zeta = 0.8
        self.ct_ctl.rbar = np.array([0.0,0.0])

        
        # Joint impedance controller
        dof = 2
        self.joint_pd      = robotcontrollers.JointPD( dof )
        self.joint_pd.kp   = np.array([  3.0, 3.0 ])
        self.joint_pd.kd   = np.array([  1.0,  1.0 ])
        
        # Effector impedance controller
        self.eff_pd      = robotcontrollers.EndEffectorPD( self.sys )
        self.eff_pd.rbar = np.array([-0.4,+0.0])
        self.eff_pd.kp   = np.array([ 25.0, 25.0 ])
        self.eff_pd.kd   = np.array([ 5.0, 5.0 ])
        

        #################
        # Memory
        #################
        
        # References Inputs
        self.user_ref             = [ 0.0 , 0.0 ]
        self.controller_mode      = 0  # Control mode of this controller node
        self.controller_mode_name = 'Initialisation'
        
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
        
        #Integral PI action
        self.last_target_position = [0.0,0.0]
        
        #Time
        self.time_now  = rospy.get_time()
        self.time_last = rospy.get_time()
        self.time_zero = rospy.get_time()
        self.controller_mode_last = self.controller_mode
        
        #################
        # Initialization
        #################
        
        # Start loop
        #self.pubish_joints_cmd_msg()
        
        # Start graphic
        self.animator = self.sys.get_animator()
        self.sys.l_domain = 0.7
        #self.animator.show( self.q )
        self.animator.show_plus( self.x , self.u, 0.0 )
        self.animator.showfig.canvas.draw()
        plt.show(block=False)
        
        # Graphic output Timer
        #self.dt2        = 0.02
        #self.timer2     = rospy.Timer( rospy.Duration( self.dt2 ), self.timed_graphic )
        
        

        
    #######################################
    def timed_controller(self):
#    def timed_controller(self, timer):
        
        ################################
        # Copmuting time values
        #################################
        if (self.controller_mode == self.controller_mode_last ):
            self.time_now  = rospy.get_time()
        else:
            # Reset time
            self.time_now  = rospy.get_time()
            self.time_zero = self.time_now 
            # Reset target position memory (only for integral velocity action)
            self.last_target_position = [self.q[1],self.q[0]+3.1415]
        
        t  = self.time_now - self.time_zero  # elapsed time in actual mode
        dt = self.time_now - self.time_last  # loop period
        
        #Memory for next loop
        self.controller_mode_last  = self.controller_mode
        self.time_last             = self.time_now #
        
        ################################
        # Control actions
        ##################################
        if (self.controller_mode == 0 ):
            self.controller_mode_name = 'disabled'
            
            # Full stop mode
            self.motors_cmd_mode = ['disable','disable']
            self.motors_cmd_pos  = [ 0.0 , 0.0 ]
            self.motors_cmd_vel  = [ 0.0 , 0.0 ]
            self.motors_cmd_tor  = [ 0.0 , 0.0 ]
            
        else:
            ##########################
            # Controllers HERE            
            ##########################
            if ( self.controller_mode == 1 ):
                self.controller_mode_name = 'manual torque control'
                
                u = np.array([ self.user_ref[0] * 1.0 , self.user_ref[1] * 1.0   ])
                
                self.motors_cmd_tor[0] = u[1]
                self.motors_cmd_tor[1] = u[0]
                
                self.motors_cmd_mode = ['torque','torque']
            
            ####################################################   
            elif  ( self.controller_mode == 2 ):
                self.controller_mode_name = 'joint velocity control'
                
                # Target velocity
                self.motors_cmd_vel[0] = self.user_ref[1] * 3.1415 * 2.0
                self.motors_cmd_vel[1] = self.user_ref[0] * 3.1415 * 1.0
                
                # Gravity compensation
                u  = self.sys.g( self.q )
                self.motors_cmd_tor[0] = u[1]
                self.motors_cmd_tor[1] = u[0]
                
                # Integral action
                self.motors_cmd_pos[0] = self.last_target_position[0] + self.motors_cmd_vel[0] * dt
                self.motors_cmd_pos[1] = self.last_target_position[1] + self.motors_cmd_vel[1] * dt
                
                self.last_target_position = self.motors_cmd_pos
                
                self.motors_cmd_mode   = ['position_velocity_torque','position_velocity_torque']
                
            ####################################################
            elif ( self.controller_mode == 3 ):
                self.controller_mode_name = 'gravity compensation'
                
                user =  np.array([ self.user_ref[0] * 1.0 , self.user_ref[1] * 1.0   ])
                
                x  = self.x 
                r  = np.array([0.0,0.0]) 
                u  = self.sys.g( self.q ) * ( 1.0 - user[0] )
                
                self.motors_cmd_tor[0] = u[1]
                self.motors_cmd_tor[1] = u[0]
                self.motors_cmd_mode = ['torque','torque']
                
                
            ####################################################    
            elif ( self.controller_mode == 4 ):
                self.controller_mode_name = 'effector velocity control'
                
                dr =  np.array([ self.user_ref[0] * 0.4 , self.user_ref[1] * 0.4   ])
                
                J = self.sys.J( self.q )
                
                dq = np.dot( np.linalg.inv( J ) , dr )
                
                # Target velocity
                self.motors_cmd_vel[0] = dq[1]
                self.motors_cmd_vel[1] = dq[0]
                
                # Gravity compensation
                u  = self.sys.g( self.q )
                self.motors_cmd_tor[0] = u[1]
                self.motors_cmd_tor[1] = u[0]
                
                # Integral action
                self.motors_cmd_pos[0] = self.last_target_position[0] + self.motors_cmd_vel[0] * dt
                self.motors_cmd_pos[1] = self.last_target_position[1] + self.motors_cmd_vel[1] * dt
                
                self.last_target_position = self.motors_cmd_pos
                
                self.motors_cmd_mode   = ['position_velocity_torque','position_velocity_torque']
                
            ####################################################
            elif ( self.controller_mode == 5 ):
                self.controller_mode_name = 'computed torque controller'
                
                user =  np.array([ self.user_ref[0] * 1.0 , self.user_ref[1] * 1.0   ])
                
                x  = self.x 
                r  = np.array([0.0,0.0]) + user * 1.0
                u  = self.ct_ctl.c( x, r, t)
                
                #print('state:',x)
                #print('cmd:',u)
                
                self.motors_cmd_tor[0] = u[1]
                self.motors_cmd_tor[1] = u[0]
                self.motors_cmd_mode = ['torque','torque']
            
            ####################################################
            elif ( self.controller_mode == 6 ):
                self.controller_mode_name = 'trajectory following'
                
                r    = self.sys.forward_kinematic_effector( self.q )
                
                # traj param
                w      = 2.0
                radius = 0.10
                center = np.array([ -0.3 , -0.2  ])
                
                r_d  = center + radius * np.array([ np.cos( w * t ) , np.sin( w * t )  ]) 
                
                dr_d = radius * w * np.array([ - np.sin( w * t ) , np.cos( w * t )  ]) 
                
                K    = np.ones( 2  ) * 1.0
                
                r_e  = r_d - r
                dr   =  dr_d + K * r_e
                
                J = self.sys.J( self.q )
                
                dq = np.dot( np.linalg.inv( J ) , dr )
                
                # Target velocity
                self.motors_cmd_vel[0] = dq[1]
                self.motors_cmd_vel[1] = dq[0]
                
                # Gravity compensation
                u  = self.sys.g( self.q )
                self.motors_cmd_tor[0] = u[1]
                self.motors_cmd_tor[1] = u[0]
                
                # Integral action
                self.motors_cmd_pos[0] = self.last_target_position[0] + self.motors_cmd_vel[0] * dt
                self.motors_cmd_pos[1] = self.last_target_position[1] + self.motors_cmd_vel[1] * dt
                
                self.last_target_position = self.motors_cmd_pos
                
                self.motors_cmd_mode   = ['position_velocity_torque','position_velocity_torque']
            
            ####################################################
            elif ( self.controller_mode == 7 ):
                self.controller_mode_name = 'empty'
                
                self.motors_cmd_tor[0] = 0.0
                self.motors_cmd_tor[1] = 0.0
                self.motors_cmd_mode = ['torque','torque']
                
            ####################################################
            elif ( self.controller_mode == 8 ):
                self.controller_mode_name = 'empty'
                
                self.motors_cmd_tor[0] = 0.0
                self.motors_cmd_tor[1] = 0.0
                self.motors_cmd_mode = ['torque','torque']
                
            ####################################################
            elif ( self.controller_mode == 9 ):
                self.controller_mode_name = 'reseting motor zero and enabling motors'

                self.motors_cmd_mode = ['enable','enable']
                
            ####################################################    
            elif ( self.controller_mode == 10):
                
                self.controller_mode_name = 'joint PD control (t-motor)'
                
                self.motors_cmd_pos[0] = self.user_ref[1] * 3.1415 * 0.5
                self.motors_cmd_pos[1] = self.user_ref[0] * 3.1415 * 0.25
                self.motors_cmd_mode   = ['position','position']
                #self.motors_cmd_mode = ['disable','disable']
                
            ####################################################    
            elif  ( self.controller_mode == 11 ):
                self.controller_mode_name = 'effector PD '
                
                user =  np.array([ self.user_ref[0] * 1.0 , self.user_ref[1] * 1.0   ])
                
                x  = self.x 
                r  = np.array([-0.4,0.0]) + user * 0.5
                u  = self.eff_pd.c( x, r, t)
                
                self.motors_cmd_tor[0] = u[1]
                self.motors_cmd_tor[1] = u[0]
                self.motors_cmd_mode = ['torque','torque']
            
            ####################################################
            elif  ( self.controller_mode == 12 ):
                self.controller_mode_name = 'joint PD control (python)'
                
                user =  np.array([ self.user_ref[0] * 1.0 , self.user_ref[1] * 1.0   ])
                
                x  = self.x 
                r  = np.array([0.0,0.0]) + user * 0.5
                u  = self.joint_pd.c( x, r, t)
                
                self.motors_cmd_tor[0] = u[1]
                self.motors_cmd_tor[1] = u[0]
                self.motors_cmd_mode = ['torque','torque']
            
            ####################################################
            elif  ( self.controller_mode == 13 ):
                self.controller_mode_name = 'effector PD control with gravity compensation'
                
                user =  np.array([ self.user_ref[0] * 1.0 , self.user_ref[1] * 1.0   ])
                
                x  = self.x 
                r  = np.array([-0.4,0.0]) + user * 0.5
                u  = self.eff_pd.c( x, r, t) + self.sys.g( self.q )
                
                self.motors_cmd_tor[0] = u[1]
                self.motors_cmd_tor[1] = u[0]
                self.motors_cmd_mode = ['torque','torque']
            
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
                
            # low-arrow
            elif (joy_msg.axes[7] < 0.0):
                
                self.controller_mode   = 10
                
            # right-arrow
            elif (joy_msg.axes[6] > 0.0):
                
                self.controller_mode   = 11
                
            # left-arrow
            elif (joy_msg.axes[6] < 0.0):
                
                self.controller_mode   = 12
                
            # up-arrow
            elif (joy_msg.axes[7] > 0.0):
                
                self.controller_mode   = 13
                

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
        
        header       = Header()
        header.stamp = rospy.Time.now()
        
        motors_msg.header   = header

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
#    def timed_graphic(self, timer):
#        
#        print('Control mode = ' , self.controller_mode_name, '  q=', self.q)
#        self.animator.show_plus_update( self.x, self.u, 0.0 )
#        plt.pause(0.01)
        
    def timed_graphic(self):
        
        print('Control mode = ' , self.controller_mode_name, '  q=', self.q)
        self.animator.show_plus_update( self.x, self.u, 0.0 )
        plt.pause(0.01)
        

def controller_thread(node):
	rate_controller = rospy.Rate(100)  # Adjust the rate for your controller
	

	while not rospy.is_shutdown():
		node.timed_controller()
		rate_controller.sleep()


#########################################

if __name__ == '__main__':
    
	plt.ion()
	matplotlib.use('TkAgg')
	#plt.ioff()
	rospy.init_node('controller',anonymous=False)
	node = robot_controller()

	#rate_controller = rospy.Rate(100)  # Adjust the rate for your controller
	controller_thread = Thread(target=controller_thread, args=(node,))	
    
	controller_thread.start()

	# Set the graphic update rate (e.g., every 0.1 seconds)
	graphic_update_period = 0.03
	graphic_last_update = time.time()

	while not rospy.is_shutdown():

		# Check if it's time to perform graphic updates
		if time.time() - graphic_last_update >= graphic_update_period:
			node.timed_graphic()
			graphic_last_update = time.time()
        
	
	controller_thread.join()
