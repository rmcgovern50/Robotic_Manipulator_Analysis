# -*- coding: utf-8 -*-
"""
Created on Sun Mar 22 11:49:26 2020
This function will take a manipulator model and work out control actions

@author: Ryan
"""

from math import sqrt
import numpy as np
import my_math as mm
from shapely.geometry import Point, shape, Polygon, LineString
import matplotlib.pyplot as plt

class path_dynamics_controller():
    """
    This class will contain the methods for designing a controller for moving
    the robot along the state space.
    """
    def __init__(self, manipulator):
        #print("initialised")
        self.bounds = manipulator.bounds
        self.joint_limits = manipulator.joint_limits
        self.boundary_points = manipulator.boundary_points
        self.s = manipulator.s
        self.sd = manipulator.sd

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

    def simulate_trajectory(self, X0, L, direction=1, T=0.01, x1_lim=1 ):
        """
        Parameters
        ----------
        X0 : (x_1, x_2)
            beginning state
        T : float
            sample size for simulation steps
        L : TYPE
            L the npercentage of actuation between U and L, value 0 < L < 1
        direction : +1 or -1, optional
            DESCRIPTION. The default is 1 representing forward integration

        Returns
        -------
        trajectory
        """
        #print(X0)
        #change default x1_lim if direction is reversed
        if direction==-1 and x1_lim ==1:
            x1_lim = 0
            
            
        state = X0
        i=0
        trajectory = [state]
        boundries = []
               
        integration_complete = False
        reason = "integration not completed yet"
        
        while(integration_complete == False and i<10000):
            
            #determine the max and min range values for the input
            D, A = self.calc_upper_lower_bound_values(state)
            x1 = state[0]
            x2 = state[1]
            
            if i == 0:
                boundries = [(D, A)]
            else:
                boundries.append((D, A))            
            
            #choose the input based on the actuation level L
            u = D + L*(A-D)
            
            delta_x1, delta_x2 = self.calc_change_s_sd(direction*x2, direction*u, T)

            x1 = x1 + delta_x1
            x2 = x2 + delta_x2
            state = (x1, x2)
            
            
            #additional logic for an early stop
            if direction == 1 and x1 > x1_lim:
                integration_complete = True
                reason = "x1>x1_lim"
                print("x1 limit crossed")
            elif direction == -1 and x1 < x1_lim:
                integration_complete = True
                reason = "x1<x1_lim"                
                print("x1 limit crossed")
                
            #with default x1_lim
            if D >= A or x1 < 0 or x1 > 1 or x2 < 0:
                   
                integration_complete = True
                if D>A:
                    reason = "L>U"
                elif x1 < 0:
                    reason = "x1<0"
                elif x1 > 1:
                    reason = "x1>1"
                elif x2 < 0:
                    reason = "x2<0"        
            else:
                trajectory.append(state)
                

            
            i=i+1

        return trajectory, boundries, reason        
    
    def find_intersect(self, trajectory_1, trajectory_2, polynomial_order=50):
        """
        this is an obsolete method
        Parameters
        ----------
        trajectory_1 : [(x1, x2),...(x1n, x2n)]
            The first trajectory
        trajectory_2 : [(x1,x2),...,(x1n, x2n)]
            the second
        the function is to return a point of intersect
        Returns
        -------
        intersection_point : (x1, x2)
            The first intersection (the on with the lowest x1 value)
        """
        import warnings
        warnings.filterwarnings("ignore")
        T1 = trajectory_1 
        T2 = trajectory_2
    
        x1 = [x[0] for x in T1]
        y1 = [x[1] for x in T1]

        x1_cast = np.array(x1, dtype=np.float32)
        y1_cast = np.array(y1, dtype=np.float32)
        z1=np.polyfit(x1_cast, y1_cast, polynomial_order)    
            
        x2 = [x[0] for x in T2]
        y2 = [x[1] for x in T2]

        x2_cast = np.array(x2, dtype=np.float32)
        y2_cast = np.array(y2, dtype=np.float32)
        z2=np.polyfit(x2_cast, y2_cast, polynomial_order) 
        
        #get range of the first trajectories x coordinates
        x1_start = x1[0]
        x1_end = x1[-1]
        if x1_start < x1_end:
            min_x1 = x1_start
            max_x1 = x1_end
        else:
            max_x1 = x1_start
            min_x1 = x1_end
        
        #get range of the second trajectories x coordinates
        x2_start = x2[0]
        x2_end = x2[-1]
        if x2_start < x2_end:
            min_x2 = x2_start
            max_x2 = x2_end
        else:
            max_x2 = x2_start
            min_x2 = x2_end
               
        #get range intersect must lie in
        if min_x1 < min_x2:
            min_x = min_x1
        else:
            min_x = min_x2
            
        if max_x1 > max_x2:
            max_x = max_x1
        else:
            max_x = max_x2
        
        min_x = np.array(min_x, dtype=np.float32)
        max_x = np.array(max_x, dtype=np.float32)
        
        #shorten the two trajectories
        #print("The intersection must be in the range ->", min_x , "to", max_x, ":)")

        #print("x_array -> ", x_intersect_range
        roots_found = np.roots((z1-z2))
        #print("roots are = ", min(roots_found[np.isreal(roots_found)]))
        
        x_intersect_range = np.linspace(min_x, max_x, 100)        
        y1_calc = np.polyval(z1, x_intersect_range)
        y2_calc = np.polyval(z2, x_intersect_range)
        plt.plot(x_intersect_range, y1_calc, ms=1)
        plt.plot(x_intersect_range, y2_calc, ms=1)
        #plt.legend(["T1", "T2"])
        plt.xlabel("$x_1$")
        plt.ylabel("$x_2$")
        plt.ylim(ymin=0)
        plt.show()  
        
        try:
            x_intersect = min(roots_found[np.isreal(roots_found)])
        except:
            x_intersect = False
        
        if  x_intersect < min_x or x_intersect > max_x or x_intersect == False:
            return False
        else:
            y_intersect = np.polyval(z1, x_intersect)
            intersection_point = (x_intersect, y_intersect)
            return intersection_point 
    

    def perform_bisection_with_L(self, initial_state, extreme_trajectories, tolerance=0.1, max_number_of_bisections=10):
        """
        Parameters
        ----------
        initial_state : (x,y)
            state to perform the bisection from
        extreme_trajectories : [Td, Ta]
            extreme trajectories to look for intersection in
        Returns
        -------
        best_trajectory : [(x1, y1),..,(xn, yn)]
            Trajectory generated by the magic L value
        magic_L : float
            magic L some number from zero to 1
        other_trajectories : [(T1),... ,(TN)]
            Trajectories from 1 to N tried before a correct one is
        L_list: [L1,...,LN]
            list of tried L values
        """
        Td = extreme_trajectories[0]
        Ta = extreme_trajectories[1]
        initial_L = 0.5
        L_upper = 1
        L_lower = 0
        L_current = initial_L
        L_list = [initial_L]
        #set the target state         
        target_state = extreme_trajectories[0][0]        
        
        distance_from_target =  tolerance + 1 #just set it to allow the algorithm to start
        
        i = 0
        #max_number_of_bisections=1
        while distance_from_target > tolerance and i  < max_number_of_bisections:
            
            #find the middle of the range
            L_current = (L_upper + L_lower)/2
            print("L value is currently ", L_current)
            
            T_current, _, _ = self.simulate_trajectory(initial_state, L_current, 1, 0.05)
            
            if i == 0:
                T_list = [T_current]
            else:
                T_list.append(T_current)
            
            #intersection_point = self.find_intersect(Td, Ta)
            intersection_point = mm.find_intersect_in_trajectories(Td, Ta)
            #print("intersection_point = ", intersection_point, target_state)
            
            
            #check for intersection point between T_current and Td
            #intersection_point = self.find_intersect(Td, T_current)
            intersection_point =  mm.find_intersect_in_trajectories(Td, T_current)
            #print("1::::::::", mm.find_intersect_in_trajectories(Td, T_current), "intersection ", intersection_point )                
            #if not false the Td is intersected else it must be Ta
            if intersection_point != False: 
                #print("intersection with Td at ", intersection_point)
                L_upper = L_current
            else:
                #intersection_point = self.find_intersect(Ta, T_current)
                intersection_point = mm.find_intersect_in_trajectories(Ta, T_current)
                #print("2:::::::::", mm.find_intersect_in_trajectories(Ta, T_current), "intersection ", intersection_point )                
                L_lower = L_current
                #deal with something weird happening 
                if intersection_point == False:
                    print("neither extreme intersected!")
                    print("there's an issue with the initial conditions most likely!")
                    print("it could also be the find_intersect function not working right")
                    return False , L_current, T_list, L_list
            
            
            delta_x = target_state[0] - intersection_point[0]
            delta_y = target_state[1] - intersection_point[1]
            distance_from_target = sqrt( (delta_x)**2 + (delta_y)**2 )
            #print("distance_from_target = ", distance_from_target, "at L = ", L_current)
            
            L_list.append(L_current)
            i = i + 1
        
        best_trajectory = T_list[-1]

        return best_trajectory, L_current, T_list, L_list
        

    def scan_input_signs_on_x1_axis(self, x1_0, x1_last, number_of_steps=10, L=1):
        
        
        x1_points_to_check = list(np.linspace(np.float32(x1_0), np.float32(x1_last), num=number_of_steps))
        x2_points_to_check = list(0*np.ones(len(x1_points_to_check)))
        
        u_DA = []
        for x1, x2 in zip(x1_points_to_check, x2_points_to_check):
            state = [x1, x2]
            D, A = self.calc_upper_lower_bound_values(state)
               
            u = D + L*(A-D) 
            
            if len(u_DA) == 0:
                u_DA = [(u, state)]
            else:
                u_DA.append((u, state))
        
        return u_DA


    def check_if_lower_bound_is_straight_line(self, extreme_trajectories):
        
        X0 = (0,0)
        Ta = extreme_trajectories[1]        
        number_of_steps = 50
        
        #get the input information along the line
        u_DA = self.scan_input_signs_on_horizontal_axis(X0, Ta, number_of_steps)
        
        As = [x[1] for x in u_DA]
        
        
        if min(As) > 0:
            
            T_line = [x[2] for x in u_DA]
            lower_bound_trajectories = T_line
            
            return lower_bound_trajectories, True
        else:
            return [] , False 
        


    def get_first_boundary_point_upward_scan(self, X0, step_size=0.1):

        #print(X0)
                

        state = X0
        D, A = self.calc_upper_lower_bound_values(state)
        
        while D<A:
            
            #split the tuple
            x1 = state[0]            
            x2 = state[1]
            #move up
            state = (x1, x2 + step_size)

            D, A = self.calc_upper_lower_bound_values((x1, x2))

            
        #print(state)
        boundary_state = state
        return boundary_state





