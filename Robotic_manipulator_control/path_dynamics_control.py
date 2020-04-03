# -*- coding: utf-8 -*-
"""
Created on Sun Mar 22 11:49:26 2020
This function will take a manipulator model and work out control actions

@author: Ryan
"""
from math import sqrt
import numpy as np
import matplotlib.pyplot as plt
from my_math import fit_curve, intersection
from my_visualising import add_to_plot

class path_dynamics_controller():
    """
    This class will contain the methods for designing a controller for moving
    the robot along the state space. I need to 
    """
    
    def __init__(self):
        print("initialised")
        #pass        



  
    def simple_time_optimal_controller(self, initial, final, bounds):
        """
        This is a method that will output a valid trajectoy of (s, sd) points 
        from which a joint torque input sequence can be derived
        It works by using a knowledge of actuator limits in the state space
        Given an intial and final condition the algorithm will try and find the fastest trajectory 
        
        Arguments:
            initial- (s_start, sd_start) initial position
            final - (s_end, sd_end) - final position
            bounds -  [(B01,B02, Ms0),(B21,B22, Ms1),(B31,B32, Ms2),...(Bn1, Bn2, Msn)]  expressions for the actuator limits
            B01 is bound 1 on actuator zero
            B02 is bound 2 on actuator zero
            Ms0 is the value that decides the upper and lower at some s, sd number
        return:
            trajectory - [(s1,sd1), ... , (sn, sdn) ]
            trajectory from s_start->s_end
        
        
        
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
        
        step_size = 0.1
        #1 backward integrate sdd = -L
        
        #perform the backward integration first from final position
        seg2 = self.integrate_motion(final, bounds, step_size, 'backwards' )
        seg2.reverse()
        
        
        #x, p1 = fit_curve(seg1)
        #plt.plot(x,p1)

        #2 forward integrate sdd == +U
        
        seg1 = self.integrate_motion(initial, bounds, step_size, 'forwards')
        
        intersection_point = self.find_intersections(seg1, seg2, step_size)
        
        all_segs = [seg1, seg2]
        
        #print(all_segs)
        
        trajectory = self.connect_trajectories(all_segs, intersection_point)
        
        
        
        
        #x, p2 = fit_curve(seg2)
        #plt.plot(x,p2)
        """
        #print(seg1)
        x_val = [x[0] for x in seg2]
        y_val = [x[1] for x in seg2]
         
        x_val = np.array(x_val, dtype=float)
        #n = np.linspace(-20,20,10)
        #print(type(x_val), type(n))
        y_val = np.array(y_val ,dtype=float)
        n = np.linspace(0, 1, 100)
        z = np.polyfit(x_val,y_val,1)
        p2 = np.poly1d(z)
        plt.plot(n, p2(n))
        
        r = np.roots(p1-p2)
        e = np.polyval(p2, r)
        #print(r, e)
        intersection = (r[0], e[0])
        print(intersection)
        plt.plot(intersection[0], intersection[1], 'or',ms=10)
        
        #plt.show()
        """
        #plot
        
        #3 backstep until tangent found sdd == -L


        #4 goto step 2
        
        
        #5 if curve formed in step one is intersected end else goto 3
        
        #6 repeat 5 until end curve is found
        
        
        
        #trajectory = 1
        #print("we got the controller designed")
        #trajectory = seg1 +seg2
        return trajectory, intersection_point
        
    def find_intersections(self, seg1, seg2, step_size):
        """
        this method takes in two lists of points and decides if they intersect
        Arguments:
            seg1 & seg2 - list of tuples [(x1, y1), ... , (xn, yn)]
        return:
                        
        """
        c = 5 #varioble to tue intersection speration
        
        i = 0
        j = 0
        
        seg1_section = []
        seg2_section = []
        
        
        while(i < len(seg1)):
        
            while(j < len(seg2)):
                
                seperation = sqrt((seg1[i][0] - seg2[j][0])**2 + (seg1[i][1] - seg2[j][1])**2)
                
                
                if seperation < c*step_size:
                                    
                    if len(seg1_section) == 0:
                        seperation_list = [seperation]
                        seg1_section = [seg1[i]]
                        seg2_section = [seg2[j]]
                        
                    else:
                        seperation_list.append(seperation)
                        seg1_section.append(seg1[i])
                        seg2_section.append(seg2[j])

                    #print(seperation, seg1[i], seg2[j])
                
                j = j + 1
            
            
            j = 0
            i = i + 1


        #val, idx = min((val, idx) for (idx, val) in enumerate(seperation_list))
        try:
            index_min = np.argmin(seperation_list)#get the closest points
        except:
            raise Exception("lines do not intersect")
            
        #print(val, idx)
        #print(seg1_section)
        #print("ewfbiuewrbgerbiuogrvb")
        #print(seg2_section)
        
        #x, p1 = fit_curve(seg1_section)
        #plt.plot(x,p1)
        
        #x, p2 = fit_curve(seg2_section)
        #plt.plot(x,p1)
        
        
        #point = intersection(p1,p2)
        
        #plt.plot(point, 'or',ms=10)
        
        #plt.show()
        return [seg1_section[index_min]]

    def connect_trajectories(self, segs, intersection):
        """
        This method takes in a few different segments and an intersection point
        
        return
            trajectory - [(s1,sd1), ... , (sn, sdn) ]
        
        """
        #print(intersection)
        
        i = 0
        j = 0
        trajectory = []
        not_finished = True
        
        while(not_finished):
            
            if len(trajectory) == 0:
                trajectory = [segs[j][i]]          
            else:
                trajectory.append(segs[j][i])   
            
            
            i = i + 1
                        
            #if we pass the intersection point
            if j == 0:
                if segs[j][i][0] >  intersection[0][0]:
                    j = j + 1 
                    i = 0
                    #increment i until interse4ction point
                    while segs[j][i][0] <  intersection[0][0]:
                        i = i + 1
                        
            
            #print(i, j, segs[j][i][0])
            if segs[j][i][0] == 1:
                not_finished = False
                    
                    
                    
        
        return trajectory


    def integrate_motion(self, pos, bounds, step_size, direction='backwards'):
        """
        Arguments:
            start_coordinate - (x, y)
            bounds -  [(B01,B02, Ms0),(B21,B22, Ms1),(B31,B32, Ms2),...(Bn1, Bn2, Msn)]  expressions for the actuator limits
                        B01 is bound 1 on actuator zero
                        B02 is bound 2 on actuator zero
                        Ms0 is the value that decides the upper and lower at some s, sd number
        return -
            trajectory - [(s1,sd1), ... , (sn, sdn) ]
        """
   
        current_pos = pos
        trajectory = [current_pos]
        i = 0
        

        integration_complete = False

        while(integration_complete == False or i>1000):
            
            #calculate -L
            #print(current_pos)
            L, U = self.get_sdd_lims(current_pos, bounds)

            if U>=L:
    
                #print(L, U)
                s = current_pos[0]
                sd = current_pos[1]
                

                #set how the integration works in if statements
                if direction == 'backwards':
                    
                    change_in_s, change_in_sd = self.calc_change_s_sd(sd, L, step_size)
                    
                    #print((change_in_s, change_in_sd, sqrt(change_in_s**2 + change_in_sd**2)))
                    new_s = s - change_in_s#c*sd
                    new_sd = sd - change_in_sd#c*L
                    
                elif direction == 'forwards':
                    
                    change_in_s, change_in_sd = self.calc_change_s_sd(sd, U, step_size)
                    
                    #print((change_in_s, change_in_sd, sqrt(change_in_s**2 + change_in_sd**2)))
                    new_s = s + change_in_s#c*sd
                    new_sd = sd + change_in_sd#c*L
                
                new_position = (new_s, new_sd)
                #check of valid
                if new_position[0] >= 0 and new_position[0] <= 1 and new_position[1] >= 0:
                    trajectory.append(new_position)
                
                
                current_pos = new_position
                i = i + 1
            else:
                integration_complete = True

            
        #print('i=',i, )



        return trajectory

    def calc_change_s_sd(self, s_vel, sd_vel, step_size):
        """
        method will take in s and sd changes and ensure that the magnitude of the step
        is equal so some stepsize
        return- delta_s (value to be added to current s)
                delta_sd (value to be added to current sd)
        
        """
        #account for all cases
        if abs(s_vel) > 0 and abs(sd_vel) > 0:
            
            length = sqrt(s_vel**2 + sd_vel**2)
            delta_s = (s_vel*step_size)/length
            delta_sd = (sd_vel*step_size)/length
            #print()
        elif  abs(s_vel) > 0 and abs(sd_vel) == 0:
            
            delta_s = step_size
            delta_sd = 0
        
        elif abs(sd_vel) > 0 and abs(s_vel) == 0:
            delta_sd = step_size
            delta_s = 0
        else:
            raise Exception("something is wrong with calc_change_sd (s_vel, sd_vel)=", (s_vel, sd_vel))


        return delta_s, delta_sd

    def get_sdd_lims(self, coordinate, bounds):
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
        s_val = coordinate[0]
        sd_val = coordinate[1]
        
        
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