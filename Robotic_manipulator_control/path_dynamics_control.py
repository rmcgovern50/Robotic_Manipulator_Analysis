# -*- coding: utf-8 -*-
"""
Created on Sun Mar 22 11:49:26 2020
This function will take a manipulator model and work out control actions





@author: Ryan
"""

class path_dynamics_control():
    """
    This class will contain the methods for designing a controller for moving
    the robot along the state space. I need to 
    """
    
    def __init__(self, robot):
        self.robot = robot
        #print("yeo")
        
    def generate_tangent_cones(self, bounds, s_lim, sd_lim):
        """
        This method will generate tangent cones that can be used to work out 
        the possible velcities along the state space of path dynamics. 
        Arguments:
            bounds = [(B01,B02, Ms0),(B21,B22, Ms1),(B31,B32, Ms2),...(Bn1, Bn2, Msn)]  expressions for the actuator limits
                    B01 is bound 1 on actuator zero
                    B02 is bound 2 on actuator zero
                    Ms0 is the value that decides the upper and lower at some s, sd number
            
        return
         tangent_cones = [[(s1, sd1), (L(s1,sd1), U(s1, sd1))]_1..., [(sn, sdn), (L(sn,sdn), U(sn, sdn))]_n]
        
        """
        s = s_lim[0]
        s_max = s_lim[1]
        s_inc = s_lim[2]

        sd = sd_lim[0]
        sd_max = s_lim[1]
        sd_inc = s_lim[2]
        
        
        
        
        while(s <= s_max):
            
            while(sd <= sd_max):
                
                print((s,sd))
                sd = sd + sd_inc
                
            sd = sd_lim[0] # reset_sd_lim
            s = s + s_inc
        
        
        
        
        
        
    
        print(self.robot.type)
        
