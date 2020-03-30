# -*- coding: utf-8 -*-
"""
Created on Sun Mar 22 11:49:26 2020
This function will take a manipulator model and work out control actions

@author: Ryan
"""


class path_dynamics_controller():
    """
    This class will contain the methods for designing a controller for moving
    the robot along the state space. I need to 
    """
    
    def __init__(self):
        print("initialised")
        #pass        

        
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
        s_max = s_lim[1]
        s_inc = s_lim[2]

        sd_val = sd_lim[0]
        sd_max = s_lim[1]
        sd_inc = s_lim[2]
        
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
        s = self.s
        sd = self.sd
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
    
    
    def dsd_ds(self, sd, sdd):
        """
        Function to calculate how d(sdot)/ds, this will essentially decide
        on the next s s dot coorcinate of any control algorithm applied 
        
        Arguments:
            sd -  ds/dt
            sdd - d^2s/dt^2
        return
            dsd/ds 
        """
        dsd_ds = abs(sqrt(sd**2 + sdd**2))
        
        #choose -ve square root
        if sdd < 0:
            dsd_ds = -1*(dsd_ds)
            
        return dsd_ds
    
    def simple_time_optimal_controller(self):#, initial, final, plot):
        """
        This is a method that will output a valid trajectoy of (s, sd) points 
        from which a joint torque input sequence can be derived
        It works by using a knowledge of actuator limits in the state space
        Given an intial and final condition the algorithm will try and find the fastest trajectory 
            
        The process is outined as:
            
            1- Start at final position and integrate backwards sdd == -L until U-L <= 0 (velocity limit curve hit) 
              or s==0 (max acceleration decelleration possible)
            
            2- Start at intial and integrate forward sdd == U until either:
                U-L <= 0 (velocity limit or inadmissable region found) or s >= 1
            
            3- If the inadmissable region is hit in step 2:
                    take steps backwards taking the sdd == L and run forwards until inadmissable region is hit or sd == 0
                    repeat until it isnt hit and at the point where the boundy of this region 
                    find the point on this curve that is closest to the in admissable region (mark it as a switching point)
            
            4- switch sdd == U at point in step 3, integrat forward until either:
                if U-L <=0: repeat step 3:
                
                else if curve formed by step 1 is intersected mark this as a switching point for decelleration
            
            5- output a valid serise of points along the trajectories calculated 
                (these can readily be used to calculater the neccessary  actuator torques)
        """
        
        #1 backward integrate sdd = -L
        
        
        
        #2 forward integrate sdd == +U
        
        
        
        #3 backstep until tangent found sdd == -L
        
        
        #4 goto step 2
        
        
        #5 if curve formed in step one is intersected end else goto 3
        
        #6 repeat 5 until end curve is found
        
        
        
        
        print("we got the controller designed")
        
        
        
        