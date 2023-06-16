#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 15 10:26:47 2023

@author: alex
"""

import numpy as np


from pyro.dynamic  import manipulator
from pyro.dynamic  import pendulum


class Proto2DoF_E23( manipulator.TwoLinkManipulator ):

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
        self.name = 'Proto 2DoF E23'
        
        # params
        self.setparams()
        
        # Graphic output
        self.l_domain = 1.
                
            
    #############################
    def setparams(self):
        """ Set model parameters here """
        
        
        self.gravity = 9.81

        self.l1      = 0.32
        self.lc1     = 0.32
        
        self.l2      = 0.4
        self.lc2     = 0.4
        
        self.I1      = 0.000025
        self.I2      = 0.000025
        self.m1      = 0.500
        self.m2      = 0.030
        
        self.d1      = 0.01
        self.d2      = 0.01
        
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
    
    sys = Proto2DoF_E23()
    
    sys.x0[0] = 0.3
    sys.x0[1] = -0.3
    
    sys.ubar[0] = 0.5
    
    sys.compute_trajectory()
    
    sys.animate_simulation()