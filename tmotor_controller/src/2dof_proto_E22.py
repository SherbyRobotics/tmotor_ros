#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 15 10:26:47 2023

@author: alex
"""

import numpy as np


from pyro.dynamic  import manipulator
from pyro.dynamic  import pendulum


class Proto2DoF_E22( manipulator.TwoLinkManipulator ):

    ########################
    def __init__(self):
        """ """
        
        # Dimensions
        dof = 2
        m   = 2
        e   = 2
               
        # initialize standard params
        manipulator.Manipulator.__init__( self, dof , m , e)
        
        # Name
        self.name = 'Proto2DoF E22'
        
        # params
        self.setparams()
        
        # Graphic output
        self.l_domain = 1.
                
            
    #############################
    def setparams(self):
        """ Set model parameters here """
        
        #self.l1  = 0.5
        #self.l2  = 0.3
        #self.lc1 = 0.2
        #self.lc2 = 0.1
        
        #self.m1 = 1
        #self.I1 = 0
        #self.m2 = 1
        #self.I2 = 0
        
        self.gravity = 9.81
        
        #self.d1 = 0.5
        #self.d2 = 0.5

        self.l1      = 0.4
        self.l2      = 0.3
        self.lc1     = 0.4
        self.lc2     = 0.3
        
        self.I1      = 0.05
        self.I2      = 0.05
        self.m1      = 0.6
        self.m2      = 0.03
        
        self.d1      = 0.0
        self.d2      = 0.0
        
        self.u_lb[0] = -1.0
        self.u_lb[1] = -1.0
        self.u_ub[0] = 1.0
        self.u_ub[1] = 1.0

    
'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":     
    
    sys = Proto2DoF_E22()
    
    sys.x0[0] = 0.3
    sys.x0[1] = -0.3
    
    sys.compute_trajectory()
    
    sys.animate_simulation()