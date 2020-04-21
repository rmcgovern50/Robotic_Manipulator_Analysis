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
    
    def __init__(self, bounds, boundry_points,s, sd):
        #print("initialised")
        #pass        
        self.bounds = bounds
        self.boundry_points = boundry_points
        self.s = s 
        self.sd = sd



        #print(bounds)
        #print("yeeeeeee")

  
    def simple_time_optimal_controller(self, initial, final):
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
            switch points = [(s1, sd1), ..., (sm, sdm)]

        """
        
        step_size = 0.05
        #1 backward integrate sdd = -L
        #print(self.bounds)
        #perform the backward integration first from final position
        

        seg_final, _, reason= self.integrate_motion_time_optimal(final, step_size, "L", 'backwards')
        seg_final.reverse()
        segments = [seg_final]

        start_seg, _, reason = self.integrate_motion_time_optimal(initial, step_size, "U",'forwards')
        segments.insert(0,start_seg)
        
        print(reason)
        
        trajectory = segments[0] + segments[1]
        combined_trajectories = False
        switch_point = False
        
        if reason == "s>1":
            combined_trajectories, switch_point = self.find_switch_point(segments[0], segments[1], step_size)
            if switch_point != False:
                print("path found")
            else:
                print("no valid path")

        elif reason == "sd<0":
            combined_trajectories, switch_point = self.find_switch_point(segments[0], segments[1], step_size)
            if switch_point != False:
                print("path found")
                trajectory = combined_trajectories
            else:
                print("no valid path")

        elif reason == "L>U": 
            combined_trajectories, switch_point = self.find_switch_point(segments[0], segments[1], step_size)
            if switch_point != False:
                print("path found") #path found algorithm complete
                trajectory = combined_trajectories
            else:
                print("bit more involved here")

                next_point = self.find_next_switch_point(segments[0][-1])

                #print(next_point)
                #work_backwards
                if next_point != False:
                    seg, _, reason= self.integrate_motion_time_optimal(next_point[0], step_size, "L", 'backwards')
                    seg.reverse()
                    combined_trajectories1, switch_point1 = self.find_switch_point(segments[0], seg, step_size)
                    
                    #work forwards
                    seg, _, reason= self.integrate_motion_time_optimal(next_point[0], step_size, "U", 'forwards')
                    combined_trajectories2, switch_point2 = self.find_switch_point(segments[0], seg, step_size)
                
                    #combined_trajectories, switch_point3 = self.find_switch_point(combined_trajectories1, combined_trajectories2, step_size)
                    if switch_point2 != False:
                        print("path found")
                        trajectory = combined_trajectories
                    else:
                        print("no valid path found")
                else:
                    print("no valid path found")
        else:
            print("something went wrong but idk what")

        return trajectory, switch_point#switch_points     

    def find_switch_point(self, seg1, seg2, step_size):

        
        switching_point_not_found = True
        switching_point = False
        const = step_size/2
        seg_1_index = 0
        seg_2_index = 0
        
        while(switching_point_not_found and seg_2_index<len(seg2)):
            
            seg_1_index = 0
            for el in seg1:
                #print(el[0], seg2[i][0], i)
                difference_x = el[0] - seg2[seg_2_index][0]
                difference_y = el[1] - seg2[seg_2_index][1]
                seg_1_index = seg_1_index + 1
                if abs(difference_x) < const and abs(difference_y) < const:
                    switching_point_not_found = False
                    switching_point = el
                    break
            
            seg_2_index = seg_2_index + 1
        
        
        if switching_point_not_found == False:
            #print((seg_1_index, seg_2_index), switching_point)
            #take last point on seg1
            point1 = seg1[seg_1_index]
            #print("hehe", seg1[seg_1_index], seg2[seg_2_index])
            #ensure the the trajectory is continuous
            while (seg2[seg_2_index][0] < point1[0]):
                seg_2_index = seg_2_index + 1
                
            #print("ho", seg1[seg_1_index], seg2[seg_2_index])
                
            combined_trajectories = seg1[0:seg_1_index] + seg2[seg_2_index:]
            return combined_trajectories, [switching_point]
        else:
            return False, False
            
        
    def find_next_switch_point(self, current_point):
        
        boundry_points = self.boundry_points
        
        s_val = current_point[0]
        
        boundry_index=0
        #find the point on the boundry next to where the boundy is intersected
        while(boundry_points[boundry_index][0] < s_val):
            boundry_index = boundry_index + 1
        

        same = False
        #increment forward and find where the velocity vector becomes a tangent to the boundry
        while(boundry_index < len(boundry_points)-1):
            
            #get current and next point
            current_b_point = boundry_points[boundry_index]
            next_b_point = boundry_points[boundry_index+1]
            
            #calculate midpoint
            midpoint = ((current_b_point[0] +  next_b_point[0])/2, (current_b_point[1] +  next_b_point[1])/2) 

            #calculate boundry gradiant
            boundry_gradient = (next_b_point[1] - current_b_point[1])/(next_b_point[0] - current_b_point[0])
            
            #get L so that the tangent vector can be calculated
            L, _ = self.calc_upper_lower_bound_values(midpoint)
            velocity_gradient = L/midpoint[1]
            
            #get value to compare gradient
            gradient_difference = boundry_gradient - velocity_gradient
            
            #check if the gradient is similar
            if abs(gradient_difference) < 0.2 :
                #if they are the same break and save point
                #print("the same")
                same = True
                break
            else:
                #print("not the same")
                boundry_index = boundry_index + 1 # increment bounry index counter
            #print(current_b_point, midpoint, next_b_point)
            #print(boundry_gradient, velocity_gradient , gradient_difference)
            
        if same == True:
            #go a little bit below the boundry for wiggle room say 5%
            tangent_point = [(current_point[0], 0.95*current_point[1])]
            return tangent_point
        else:
            return False
        
    def integrate_motion_time_optimal(self, pos, step_size, acceleration_choice, direction="backwards"):
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
               
        trajectory = [pos]
        boundries = []
        i = 0
        
        s = pos[0]
        sd = pos[1]
    
        integration_complete = False
        reason = "integration not completed yet"
        
        while(integration_complete == False and i<1000):
            
            #calculate -L
            #print(current_pos)
            L, U = self.calc_upper_lower_bound_values(pos)
            s = pos[0]
            sd = pos[1]
            
            if i == 0:
                boundries = [(L, U)]
            else:
                boundries.append((L, U))            


            if acceleration_choice == "U":
                up_down_acc = U
            elif acceleration_choice == "L":
                 up_down_acc = L 
            
            #print(L, U)
            if direction == "backwards":
                delta_s, delta_sd = self.calc_change_s_sd(-sd, -up_down_acc, step_size)#integrate backwards
            elif direction == "forwards":
                delta_s, delta_sd = self.calc_change_s_sd(sd, up_down_acc, step_size)#integrate forwards

            s = s + delta_s
            sd = sd + delta_sd
            pos = (s, sd)
            #print(pos)

                        #print(L, U)
            
            if L > U or s < 0 or s > 1 or sd < 0:
                integration_complete = True
                if L>U:
                    reason = "L>U"
                elif s < 0:
                    reason = "s<0"
                elif s > 1:
                    reason = "s>1"
                elif sd < 0:
                    reason = "sd<0"
                
                
                    
            else:
                trajectory.append(pos)                
            #print(trajectory)
            i=i+1
        #print('i=',i, )
        #print(len(trajectory))
        #print("==================")
        #print(len(boundries))
        #print("==================")
        #print(boundries)
        #print(trajectory)
        return trajectory, boundries, reason

        


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
            b = step_size/length
            
            delta_s = (s_vel*b)#step_size)/length
            delta_sd = (sd_vel*b)#step_size)/length
            
            #c = sqrt(delta_s**2 + delta_sd**2)
            
            #print(c, length, delta_s, delta_sd)
        elif  abs(s_vel) > 0 and abs(sd_vel) == 0:
            
            delta_s = step_size
            delta_sd = 0
        
        elif abs(sd_vel) > 0 and abs(s_vel) == 0:
            delta_sd = step_size
            delta_s = 0
        else:
            raise Exception("something is wrong with calc_change_sd (s_vel, sd_vel)=", (s_vel, sd_vel))


        return delta_s, delta_sd


    def calc_upper_lower_bound_values(self, point):
        """
        Method to take in a point and evaluate the upper and lower bounds at that point
        Arguments:
            point = (s, sd)
        """
        L = point[0]
        U = point[1]
        bounds = self.bounds
        i = 0

        
        for bound in bounds:
            
            lim_direction_decider = bound[2].subs([(self.s, point[0]), (self.sd, point[1])])
            sub_list = [(self.s, point[0]), (self.sd, point[1])]
            
            #put the bounds the right way round

            if lim_direction_decider > 0:
                L_to_check =  bound[0].subs(sub_list)
                U_to_check = bound[1].subs(sub_list)
              
            elif lim_direction_decider < 0:
                L_to_check =  bound[1].subs(sub_list)
                U_to_check = bound[0].subs(sub_list)
                
            else:
                raise Exception("M(s) cannot be equal to zero - error in calc_upper_lower_bound_values method")
            
            
            
            if i == 0:
                L = L_to_check
                U = U_to_check
            else:
                if L_to_check > L:
                    L = L_to_check
                if U_to_check < U:
                    U = U_to_check
            i = i + 1
            
            
        return L, U

    def evaluate_Ek_s(self, Ek_s, s, sd):
        """
        Method to take in the kinetic energy function of teh robot and oyhre
        
            Arguments:
                Ek_s  and expression of teh form f(s, sd)
        
            return
                Ek_val = Ek_evaluated at (s,sd)
        """
        Ek_val = Ek_s.subs([(self.s, s), (self.sd, sd)])
        
        
        
        return Ek_val
























