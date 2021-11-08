# -*- coding: utf-8 -*-
"""
Created on Sun Mar 22 11:49:26 2020
This function will take a manipulator model and work out control actions

@author: Ryan
"""

from math import sqrt
import numpy as np
from my_math import find_intersection_points
from shapely.geometry import Point, shape, Polygon, LineString

import matplotlib.pyplot as plt

class path_dynamics_controller():
    """
    This class will contain the methods for designing a controller for moving
    the robot along the state space. I need to 
    """
    
    def __init__(self, manipulator):
        #print("initialised")
        #pass        
        self.bounds = manipulator.bounds
        self.joint_limits = manipulator.joint_limits
        self.boundary_points = manipulator.boundary_points
        self.s = manipulator.s 
        self.sd = manipulator.sd
        



    def safe_time_optimal_control(self, initial, final):
        """
        Method to navigate the admissible region without violating a potential energy constrain in addition to the
        constraints given by the actuator
        
        Arguments:
            initial- (s_start, sd_start) initial position
            final - (s_end, sd_end) - final position
            
            implicit
                bounds -  [(B01,B02, Ms0),(B21,B22, Ms1),(B31,B32, Ms2),...(Bn1, Bn2, Msn)]  expressions for the actuator limits
                B01 is bound 1 on actuator zero
                B02 is bound 2 on actuator zero
                Ms0 is the value that decides the upper and lower at some s, sd number
                
                energy bounds
            
        return:
            trajectory - [(s1,sd1), ... , (sn, sdn) ]
            trajectory from s_start ---> s_end
            switch points = [(s1, sd1), ..., (sm, sdm)]
        """
        
        
        
        
        
        print("this is some function")
        """
        bang bang trajectory
        
        integrate back from final until bounds violated or s < 0
        
        integrate forward from initial until bounds violated or s > 1
        
        check for intersection
            if intersection found save it and return answer
            
            if not - fail
        
        
        """
        print("return answer")


    def simple_time_optimal_controller(self, initial, final):
        """
        This is a method that will output a valid trajectory of (s, sd) points 
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
                trajectory = combined_trajectories
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
        """
        method to find the point on 2 trajectories that intersect, return the switch point and combined trajectory
        
        """
        
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
            seg_1_index = seg_1_index - 1
            point1 = seg1[seg_1_index]
            #print("hehe", seg1[seg_1_index], seg2[seg_2_index])
            #ensure the the trajectory is continuous
            while (seg2[seg_2_index][0] < point1[0]):
                seg_2_index = seg_2_index + 1
                
            #print("ho", seg1[seg_1_index], seg2[seg_2_index])
            #print("hmm", seg2[seg_2_index:])
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
    def generate_time_optimal_trajectory(self, initial_coordinate, final_coordinate):
        """
            method to take steps neccesary to design a time opeitmal control trajectory
            
            return:
                trajectory - [(s1, sd1),...,(sn,sdn) ]
                switching_points - [(swx1, swy1),...,(swxn,swyn)]
        """



        #Ek_q = pd.get_machine_kinetic_energy_q(self)[0]
        #get it in terms of s            
        #Ek_s = Ek_q.subs([(self.q1, self.qs[0]), (self.q2, self.qs[1]), (self.q1d, self.qds[0]), (self.q2d, self.qds[1])])

        trajectory, switching_points = self.simple_time_optimal_controller(initial_coordinate, final_coordinate)

        return trajectory, switching_points


#==========================polynomial approaches===================================================



    def polynomial_constraint_based_time_optimal_control(self, z, x,\
                                                         s_start = 0, sd_start = 0,\
                                                         s_end = 1, sd_end  = 0,\
                                                         integration_step = 0.1,\
                                                         min_arc_length = 1,\
                                                         binary_search_multiplier = 1):

        
        #====================step 1===================================
        #integrate backward from end until constraint violation
        #this is section_end
        
        #====================step 2====================================      
        #intergrate forward from start until constraint violation
        #check for intersection with section_end
        #if intersect save switching point and compile control trajectory
        #if not go to next step
        #====================step 3====================================
        #lower the value of sdot by some increment and integrate forward with max deceleration
        #if the limit curve is violated reduce sdot and repeat until it is not

        #====================step 4==========================================
        #integrate backward from the new sdot point until the previous section is met
        #save this as a section of the curve

        #====================step 5==========================================
        #goto step 2 and repeat until a path is found
        # after some number of iterations consider path impossible        
        
        """
        constraint sets made from:
            s_current < s_final #path end constraint
            s_current > 0 #path start constraint 
            sdot_current < np.polyval(z, s_current) #abstract_constraints and actuator constraints           
        """        
        
        #=============save inputs=======================        
        input_integration_step = integration_step
        
        #===============================================
        print("Finding_time_optimal_path..")         
        s = s_end
        sd = sd_end
        intersection_list = [()]
        path_complete = False
        
        end = (s_end, sd_end)
        _, segment_end = self.integrate(end, integration_step, z,"L", "backwards") #integrate from end backwards assuming a max deceleration is used to get there
        print("last segment found")         
        start = (s_start, sd_start)       
        _, segment_start = self.integrate(start, integration_step, z,"U", "forwards")
        print("first segment found")   
        #trajectory, i = self.find_switch_point(segment_start, segment_end, 0.05)
        x, y = find_intersection_points(segment_start, segment_end)
        

        if x != False and y != False:
            #intersection is found a solution is found
            x = x[-1]
            y = y[-1]
            intersection_list = [(x, y)] #put in the correct format
            seg_list = [segment_start, segment_end]
            trajectory = self.combine_trajectories(seg_list, intersection_list) #combine the trajectories
            control_trajectory =  trajectory
        else:   
            end_found = False
            seg_counter = 0
            seg_list = [segment_start]
            no_valid_path = False
            i=0
            
            while end_found == False:
                
                back_intersection_found = False
                error_counter = 0 #this will stop us attempting too many times
                binary_search_resoution_delta_sd = binary_search_multiplier*input_integration_step#this will set how much the binary search must converge before it is considered finished
                    
                while back_intersection_found == False and no_valid_path == False:
                    
                    current_segment =  seg_list[seg_counter]
                    current_point = current_segment[-1]#last point on the current segment 
                    violation = 0 # (velocity limit curve collided with)
                    sd_collision_point = current_point[1]
                    #=====================perform binary search=====================================
                    
                    sd_constraint_boundary = current_point[1]
                    #initialise points for a binary search
                    upper_point_sd = sd_constraint_boundary
                    lower_point_sd = 0
                    midpoint_sd = (upper_point_sd + lower_point_sd)/2#calculate midpoint
                    point_close_to_boundary_found = False

                    j = 0
                    
                    
                    while point_close_to_boundary_found == False:
                        prev_sd = midpoint_sd
                        midpoint = (current_point[0], prev_sd)#set midpoint to integrate from
                        violation, new_seg = self.integrate(midpoint, integration_step, z, "L", "forwards")#integrate until violation is hit
                        
                        if violation == 0:#safety constraint violated
                            upper_point_sd = midpoint_sd
                        else: #safety constraint not violated
                            lower_point_sd = midpoint_sd
                            
                        midpoint_sd = (upper_point_sd + lower_point_sd)/2#calculate midpoint
                        change_sd = prev_sd - midpoint_sd#calculate change in sd
                        #print("change in sd", abs(change_sd), j)
                        
                        if violation != 0 and abs(change_sd) < binary_search_resoution_delta_sd:
                            point_close_to_boundary_found = True
                        j = j  + 1
                        
                    current_point = (current_point[0], prev_sd)
                    #print(current_point)
                    
                    #========Find point on the constraint curve closest to the arc=================
                    s_val = [x[0] for x in new_seg]
                    sd_val_control_arc = [x[1] for x in new_seg]
                    sd_val_constraint_curve = np.polyval(z, s_val)
    
                    difference = np.array(sd_val_constraint_curve) - np.array(sd_val_control_arc)
                    min_index_result = np.where(difference == np.amin(difference))
                    min_index = min_index_result[0][0] #save the index of the minimum result
                                    
                    if min_index < min_arc_length:
                        try:
                            current_point = new_seg[min_arc_length] #set this point
                        except:
                            print("arc length of backward integration switch point could not be set to a guarantee the min arc length of ", min_arc_length)
                            print("the switch point is set to guarantee ", len(sd_val_control_arc), " points on arc")
                            index = len(sd_val_control_arc) - 1
                            current_point = new_seg[index] #set this point
    
                    #==============================================================================
                    #print(current_point)
                    _, new_seg = self.integrate(current_point, integration_step, z, "L", "backwards")
                    #print(new_seg[0], len(new_seg))
                    #We know they will intersect so obtain point
                    x, y = find_intersection_points(current_segment, new_seg) 
                                       
                    try:
                        intersection = (x[-1], y[-1])#format the intersection points
                        print("intersection point found: ", (x, y))  
                        if seg_counter == 0:
                            intersection_list = [intersection]
                        else:
                            intersection_list.append(intersection)
                        back_intersection_found = True
                    except:
                        #============attempt to still make it to the end before deciding the path is invalid==================
                        error_counter=error_counter + 1
                        integration_step = input_integration_step/(error_counter+1)#this will set how much the binary search must converge before it is considered finished
                        print("k", error_counter, integration_step)
                        if error_counter<2:
                            print("Intersection has failed to be found ", error_counter,"  time(s) (likely due to resolution of the integration)")
                            print("decreasing integration step size to ", integration_step)
                        else:
                            print("no valid path found")
                            #print("k", binary_search_resoution_delta_sd, sd_collision_point)
                            seg_list.append(new_seg)
                            no_valid_path = True
                        
                        #===================================

                #if no problems were found generating the backward arc move on
                if no_valid_path == False:
                    #print("here")
                    integration_step = input_integration_step
                    seg_list.append(new_seg)
                    seg_counter = seg_counter + 1    
                    
                    current_point = seg_list[seg_counter][-1] #get last element of list
                    intersection_list.append(current_point) # this point is always a switching point
                    
                    violation, new_seg = self.integrate(current_point, integration_step, z, "U", "forwards") #forward integration until violation is hit
    
                    seg_list.append(new_seg)
                    seg_counter = seg_counter + 1  
                    
                    #check for intersection with the last section
                    x, y = find_intersection_points(seg_list[-1], segment_end) 
                    if x != False or y != False:
                        intersection = (x[-1],y[-1])
                        #print("end found intersect :", intersection)
                        intersection_list.append(intersection)
                        end_found = True
                    print("segements found - ", seg_counter)
                    
                    if len(seg_list) != len(intersection_list) +1 and end_found == False:
                        print("problem found", len(seg_list), len(intersection_list))
                    
                    i = i + 1
                elif no_valid_path == True:
                    #the algorithm cannot proceed so break the loop
                    end_found = True
                
            seg_list.append(segment_end)
            seg_counter = seg_counter + 1

            #print("list lengths", len(intersection_list), len(seg_list))
            
            try:
                control_trajectory = self.combine_trajectories(seg_list, intersection_list)
            except:
                print("A problem was encountered combining the control trajectories")
                print("number of segments = ", len(seg_list),"number of switching points = ", len(intersection_list) )
                control_trajectory = [j for i in seg_list for j in i]
                
            path_complete = not no_valid_path #return true if complete path found
            
        return control_trajectory, intersection_list, path_complete

    def check_all_constraints(self, s, sdot, z, s_end, s_start):
        """
            method that will evalate each constraint at each point and checky if they are exceeded 
        """
        
        sdot_calc = np.polyval(z, s)
        if sdot < sdot_calc and sdot > 0 and s < s_end and s > s_start:
            contraints_violated = False
        else:
            contraints_violated = True
        return contraints_violated

    def integrate(self, pos, step_size, polynomial_coefficents, acceleration_choice, direction):
        """
        Arguments:
            pos - (s, sd) tuple of numbers
            step_size - float that decides how big each integration step should be.
            acceleration_choice - this is a string "U" will give max possible acceleration
                                                   "L" will give min possible acceleration
                                  add functionality for other accelerations
            direction - string giving the direction of the integration
        return:
            trajectory - [(s1,sd1), ... , (sn, sdn)]
            violations-                 
                if sd> np.polyval(polynomial_coefficents, s):
                    violation = 0 #"Safety_Constraint_Volation"
                elif s < 0:
                    violation = 1 #"s<0"
                elif s > 1:
                    violation = 2 #"s>1"
                elif sd < 0:
                    violation = 3 #"sd<0"
        """

        trajectory = [pos]
        boundries = []
        i = 0
        
        s = pos[0]
        sd = pos[1]
    
        integration_complete = False
        violation = "integration not completed yet"
        
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
            #print(s, sd, np.polyval(polynomial_coefficents, s))
            
            if sd > np.polyval(polynomial_coefficents, s) or s < 0 or s > 1 or sd < 0:
                integration_complete = True
                if sd> np.polyval(polynomial_coefficents, s):
                    violation = 0#"Safety_Constraint_Volation"
                elif s < 0:
                    violation = 1#"s<0"
                elif s > 1:
                    violation = 2#"s>1"
                elif sd < 0:
                    violation = 3#"sd<0"
   
            else:
                trajectory.append(pos)                
            #print(trajectory)
            i=i+1
            
        if direction == "backwards":#reverse backwards trajectories
             trajectory.reverse()
            
        return violation, trajectory

    def combine_trajectories(self, segment_list, intersection_list):
        """
        method to combine a list of n segments with n-1 intersctions into a single trajectory    
        
        """
        seg_counter = 0
        intersection_counter = seg_counter #initialise as the same
        
        #loop to move along all elements
        #===============loop one=================
        #cut seg 1 at intersection 1, cut seg 2 at intersection 1
        #cut seg 2 at intersection 2, cut seg 3 at intersection 2
        new_segment_list = []
        i = 0
        
        
        while(seg_counter < len(segment_list) and intersection_counter < len(intersection_list)):
            
            current_seg = segment_list[seg_counter]
            intersection = intersection_list[intersection_counter]
            curr_el = 0
                           
            
            #find all valid points on the list up until the switching point
            while current_seg[curr_el][0] < intersection[0]:
                curr_el = curr_el + 1
            
            #save the valid section in the new new_segment_list
            if i == 0:
                new_segment_list = [current_seg[0:curr_el]]
            else:   
                new_segment_list.append(current_seg[0:curr_el]) #replace the segment with the cut one
                
            seg_counter=seg_counter+1
            
            #===============
            #select the next segment
            next_seg = segment_list[seg_counter]
            curr_el = len(next_seg) -1#last element
            
            #loop backwards until the intersection is collided with
            while next_seg[curr_el][0] >= intersection[0]:
                curr_el = curr_el - 1            
            
            #save the valid section of the trajectory after the intersection
            new_segment_list.append(next_seg[curr_el+1:])
            #================
            
            intersection_counter = intersection_counter + 2#increment to next trimmable point
        
            """
            #select the next segment
            next_seg = segment_list[seg_counter]
            curr_el = len(next_seg) -1#last element
            
            #loop backwards until the intersection is collided with
            while next_seg[curr_el][0] > intersection[0]:
                curr_el = curr_el - 1            
            
            #save the valid section of the trajectory after the intersection
            new_segment_list.append(next_seg[curr_el:])
            seg_counter=seg_counter+1 #increment the section counter
            """
            seg_counter=seg_counter+1
            #loop counter
            i = i + 1
            
        trajectory = [j for i in new_segment_list for j in i]
        return trajectory
        
        
        
        
    def length_of_arc_until_intersect(self, seg, s_intersect):
        """
        counts the distance until the last section using knowlege of the 
        current section and intersection with the last one
        """
        i=len(seg) - 1
        count = 0 
        #print("seg: ", seg)
        #print("inter", s_intersect)
        try:
            while(seg[i][0] >  s_intersect):
                i = i - 1
                count = count + 1
            return count
        except:
            
            return 0 # no elements cross

        


    def backward_reachability_from_point(self, target_point, polygons, plot=True):
        """
        This method will start at the end point, choose a polygon form polygons
        that contains the end point, and perform a backward integration until the bounds of this polygon are collided with
        when this happens for two arcs the section inbetween will be returned as a valid boundary
        """
        
        #print(polygons)
        i = 0

        point_2_check = Point(target_point[0],target_point[1])
        
        for p in polygons:
            in_p = p.contains(point_2_check)
            if in_p:
                break
            
            i = i + 1
        p = polygons[i]
        in_p = p.contains(point_2_check)
        

        #form the two extreme bounding arcs
        t = self.dp_integrate(target_point, 0.5, "L", "backwards", p)
        t2 = self.dp_integrate(target_point, 0.5, "U", "backwards", p)
        
        pnp1 = t[0]
        pnp1_s = pnp1[0]
        pnp1_sd = pnp1[1]

        pnp2 = t2[0]
        pnp2_s = pnp2[0]
        pnp2_sd = pnp2[1]       
        
        polynomial_boundary =  list(p.exterior.coords)
        
        s_point1_v =pnp1_s*np.ones(len(polynomial_boundary))
        sd_point1_v =pnp1_sd*np.ones(len(polynomial_boundary))          

        s_point2_v =pnp2_s*np.ones(len(polynomial_boundary))
        sd_point2_v =pnp2_sd*np.ones(len(polynomial_boundary))          
                
        s_boundary = np.array([x[0] for x in polynomial_boundary])
        sd_boundary = np.array([x[1] for x in polynomial_boundary])
        
        
        distances1 = np.square(s_point1_v - s_boundary) + np.square(sd_point1_v - sd_boundary)
        min_distance1 = np.min(distances1)
        min_distance_index1 = np.argmin(distances1)
        
        distances2 = np.square(s_point2_v - s_boundary) + np.square(sd_point2_v - sd_boundary)
        min_distance2 = np.min(distances2)
        min_distance_index2 = np.argmin(distances2)
        
        print("distance and array index for 1", min_distance1, min_distance_index1)
        print("distance and array index for 2", min_distance2, min_distance_index2)       
      
        
        #need to ensure correct half is chosen here 
        #print(sd_point_v)
        
        
        
        
        
        if plot:
            
            
            
            
            x1 = [x[0] for x in list(p.exterior.coords)]
            y1 = [x[1] for x in list(p.exterior.coords)]
            #print(len(list(p3.exterior.coords)))
            plt.plot(x1, y1, color='c')
            plt.plot(x1[656:], y1[656:], color='g')
            plt.plot(x1[0:126], y1[0:126], color='g')    
            
            
            xarc1 = [x[0] for x in t]
            yarc1 = [x[1] for x in t]
            #print(len(list(p3.exterior.coords)))
            plt.plot(xarc1,yarc1, color='g')

            xarc2 = [x[0] for x in t2]
            yarc2 = [x[1] for x in t2]
            #print(len(list(p3.exterior.coords)))
            plt.plot(xarc2, yarc2, color='g') 

           
            xs = [point_2_check.x]
            ys = [point_2_check.y]
            plt.plot(xs, ys, 'or', color='c')
            plt.show()
        
        #return region_of_stabilizability

    def dp_integrate(self, pos, step_size, acceleration_choice, direction, polygon):
        """
        integrate until point is no longer inside polygon
        Arguments:
            pos - (s, sd) tuple of numbers
            step_size - float that decides how big each integration step should be.
            acceleration_choice - this is a string "U" will give max possible acceleration
                                                   "L" will give min possible acceleration
                                  add functionality for other accelerations
            direction - string giving the direction of the integration
        return:
            trajectory - [(s1,sd1), ... , (sn, sdn)]
        """

        trajectory = [pos]
        boundries = []
        i = 0
        
        s = pos[0]
        sd = pos[1]
    
        integration_complete = False
        violation = "integration not completed yet"
        
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
            point_2_check = Point(s, sd)
            in_p = polygon.contains(point_2_check)
            
            if not in_p:
                integration_complete = True
               
            else:
                trajectory.append(pos)                
            #print(trajectory)
            i=i+1
            
        if direction == "backwards":#reverse backwards trajectories
             trajectory.reverse()
            
        return trajectory



