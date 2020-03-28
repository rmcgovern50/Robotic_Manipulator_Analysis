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
        
    def generate_tangent_cone_components(self, bounds, s_lim, sd_lim):
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
        s_val = s_lim[0]
        s_max = 1#s_lim[1]
        s_inc = 0.5#s_lim[2]

        sd_val = sd_lim[0]
        sd_max = 1#s_lim[1]
        sd_inc = 0.5#s_lim[2]
        
        tangent_cones = []
        tangent_cone = []
        
        #while loops loop through a regon of the parameterised state space
        while(s_val <= s_max):
            
            while(sd_val <= sd_max):
                
                #evaluate numerically
                L, U = self.get_min_upper_max_lower(s_val, sd_val, bounds)                
                tangent_cone = [(s_val, sd_val), L, U]
                
                if s_val == 0:
                    tangent_cones = [tangent_cone]
                else:
                    tangent_cones.append(tangent_cone)
                #print(upper_lim)
                
                #print(bounds)
                sd_val = sd_val + sd_inc
                
            sd_val = sd_lim[0] # reset_sd_lim
            s_val = s_val + s_inc
        
        #print(tangent_cones)
        return tangent_cones
        #print(self.robot.type)
        
    def get_min_upper_max_lower(self, s_val, sd_val, bounds):
        """
        This is a method that will simply take in a coordnate in the state space and
        output the up and down component of the tangent cone at that point
        
        Arguments:
            s_val -s coordinate
            sd_val - sd coordinate
            bounds -  [(B01,B02, Ms0),(B21,B22, Ms1),(B31,B32, Ms2),...(Bn1, Bn2, Msn)]  expressions for the actuator limits
                        B01 is bound 1 on actuator zero
                        B02 is bound 2 on actuator zero
                        Ms0 is the value that decides the upper and lower at some s, sd number
                        
        return :
            L - the max lower limit of all actuatators at (s_val, sd_val)
            U - the min upper limit of all actuatators at (s_val, sd_val)
        """
        
        #print(len(bounds))
        number_of_actuators = len(bounds)
        i = 0
        s = self.robot.s
        sd = self.robot.sd
        upper = 0
        lower = 0
        
        
        
        #loop through all the actuators and evaluate the bounds
        while(i < number_of_actuators):
            #evaluate the bounds
            B1 = bounds[i][0].subs([(s, s_val), (sd, sd_val)])
            B2 = bounds[i][1].subs([(s, s_val), (sd, sd_val)])
            dir_decider = bounds[i][2].subs([(s, s_val), (sd, sd_val)])
            
            if dir_decider > 0:
                #this is how we get the min upper each loop always makes the upper lim the lowest of the actuator lims
                #
                if B2 < upper  or i ==0: 
                    upper_lim = B2
                #similar;y take the max lower lim
                if B1 > lower or i ==0:
                    lower_lim = B1
                
            elif dir_decider < 0:
                #this is how we get the min upper each loop always makes the upper lim the lowest of the actuator lims
                #
                if B1 < upper  or i ==0: 
                    upper_lim = B1
                #similar;y take the max lower lim
                if B2 > lower or i ==0:
                    lower_lim = B2
                
            else:
                raise Exception("Problem with M(s), this value cannot be zero check things over bounds are ->", (B1, B2)) 
            
            #print((lower_lim, upper_lim))
            i = i + 1

        L = lower_lim
        U = upper_lim
        #print(L, U)
        return L, U