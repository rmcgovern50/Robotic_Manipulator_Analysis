# -*- coding: utf-8 -*-
"""
Created on Sun Mar 22 11:49:26 2020
This function will take a manipulator model and work out control actions

@author: Ryan
"""

from math import sqrt, exp
import numpy as np
import my_math as mm
from shapely.geometry import Point, shape, Polygon, LineString
import matplotlib.pyplot as plt
from sympy import symbols
from my_sorting import combine_to_tuples

class path_dynamics_controller():
    """
    This class will contain the methods for designing a controller for moving
    the robot along the state space.
    """
    def __init__(self, manipulator):
        #print("initialised")
        self.manipulator = manipulator
        self.eng = manipulator.eng
        #self.bounds = manipulator.bounds
        #self.joint_limits = manipulator.joint_limits
        self.boundary_points = manipulator.boundary_points
        self.s = manipulator.x1
        self.sd = manipulator.x2
        self.constraint_list = self.form_constraint_list()
        self.x1_lims = manipulator.x1_lims
        self.x2_lims = manipulator.x2_lims
        self.x1_sym, self.x2_sym\
        = symbols('x1 x2')


    def form_constraint_list(self):
        """
        Form list of sympy expressions for the system constraints
        If any of these are greater than zero the constraint is violated
        """
        x1, x2, Ax, Dx\
        = symbols('x1 x2 Ax Dx')
        constraints = [-x1 + 0]
        constraints.append(x1 - 1)
        constraints.append(-x2 + 0)
        constraints.append(Dx - Ax)

        return constraints


    def calc_change_s_sd(self, s_vel, sd_vel, step_size):
        """
        method will take in s and sd changes and ensure that the magnitude of the step
        is equal so some stepsize
        return- delta_s (value to be added to current s)
                delta_sd (value to be added to current sd)        
        """
        try:
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
        except:
            print("I can see inside now!!!!")
            
        return delta_s, delta_sd


    def calc_upper_lower_bound_values(self, point):
        """
        Method to take in a point and evaluate the upper and lower bounds at that point
        Arguments:
            point = (s, sd)
        """
        eng = self.eng
        #print(point, type(point))
        
        x1 = float(point[0])
        x2 = float(point[1])
        
        qx1_evaluated, dqx1_evaluated, ddqx1_evaluated =\
            self.manipulator.evaluate_path_parameters(x1, x2)
        
        limits = eng.calc_A_D("universalUR5",\
                              eng.double(x1),\
                              eng.double(x2),\
                              qx1_evaluated,\
                              dqx1_evaluated,\
                              ddqx1_evaluated)
        
        A = limits[0][0]
        D = limits[0][1]
        return A, D

    def simulate_trajectory(self, X0, L, direction=1, T=0.01, x1_lim=1):
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
        
        x2_max = self.x2_lims[1]
        
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
            #print("i = ", i)
            #determine the max and min range values for the input
            A, D = self.calc_upper_lower_bound_values(state)
            #print((i/5000)*100, " % complete", (A, D))
            x1 = state[0]
            x2 = state[1]
            
            if i == 0:
                boundries = [(D, A)]
            else:
                boundries.append((D, A))            
            
            if D <= A:
                #choose the input based on the actuation level L
                u = D + L*(A-D)
                try:
                    delta_x1, delta_x2 = self.calc_change_s_sd(direction*x2, direction*u, T)
                except:
                    print("I gotcha boy")
                x1 = x1 + delta_x1
                x2 = x2 + delta_x2
                state = (x1, x2)
                #print("state: ", state)
                violated, constraint =\
                    self.check_if_constraints_violated_analytically(state) 
                
                if violated == True:
                    integration_complete = True
                    if constraint == 1:
                        reason = "x1<x1_lower_lim"
                    elif constraint == 2:
                        reason = "x1>x1_upper_lim"                
                    elif constraint == 3:
                        reason = "x2<x2_lower_lim"                     
                    elif constraint ==4:
                        reason = "L>U"
                    elif constraint > 4:
                        reason = "additional constraint violated" 
                else:
                    trajectory.append(state)
            else:
                integration_complete = True
                
            i=i+1

        return trajectory, boundries, reason        
    
    
    def simulate_trajectory_with_control(self, X0, L, direction=1, T=0.01):

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

        state = X0
        i=0
        trajectory = [state]
        boundries = []
        u_list = []
        integration_complete = False
        reason = "integration not completed yet"
        
        """
        directly use the passed in L value
        """
        if isinstance(L, float):
            control_strategy="raw"
        
        #elif apply some form of control
        elif isinstance(L, list):
            
            control_strategy =L[0]
            if control_strategy=="1":
                lower_bound = L[1]
                upper_bound = L[2]
        else:
            print("something is wrong with input L")
        
        
        while(integration_complete == False):
            
            #determine the max and min range values for the input
            try:
                A, D = self.calc_upper_lower_bound_values(state)
            except:
                print("problem , trajectory lenghh is", len(trajectory))
                break
            x1 = state[0]
            x2 = state[1]
            
            if i == 0:
                boundries = [(D, A)]
            else:
                boundries.append((D, A))            
            """
            ================input control=============================
            """
            #choose the input based on the actuation level L
            if control_strategy == "raw":
                 pass#directly use L
            elif control_strategy == "1":
                """
                find closest x1 point on lower bound (binary search)
                """
                close_index = self.x1_binary_search(x1, lower_bound)
                x2_lower = lower_bound[close_index][1]
                """
                find closest x1 point on the upper bound (binary search)
                """
                close_index = self.x1_binary_search(x1, upper_bound)
                x2_upper = upper_bound[close_index][1]
                L = self.lambda_raw_linear_interpolation(x2, x2_upper, x2_lower)
            elif control_strategy=="2":
                L = self.lambda_simple_linear_saturation(state)
            elif control_strategy=="3":
                L = self.lambda_simple_quadratic_saturation(state)
            elif control_strategy=="4":
                L = self.lambda_single_guide_saturation(state)
            elif control_strategy=="5":
                L = self.lambda_single_guide_2_saturation(state)
            elif control_strategy== "6":
                L = self.lambda_different_guide_saturation(state)
            elif control_strategy== "7":
                L = self.lambda_single_guide_3_saturation(state)
            """
            ================================================================
            """
            u = D + L*(A-D)
            
            if len(u_list)== 0:
                u_list = [u]
                L_list = [L]
                A_list = [A] 
                D_list = [D]
                u_list_with_state = [(u, state)]
                L_list_with_state = [(L, state)]
                A_list_with_state = [(A, state)] 
                D_list_with_state = [(D, state)]
            else:
                u_list.append(u)
                L_list.append(L)
                A_list.append(A)
                D_list.append(D)
                u_list_with_state.append((u, state))
                L_list_with_state.append((L, state))
                A_list_with_state.append((A, state))
                D_list_with_state.append((D, state))

            delta_x1, delta_x2 = self.calc_change_s_sd(direction*x2, direction*u, T)

            x1 = x1 + delta_x1
            x2 = x2 + delta_x2
            state = (x1, x2)
            print("control state: ", state)
            violated, constraint =\
                self.check_if_constraints_violated_analytically(state) 
            
            if violated == True:
                integration_complete = True
                if constraint == 1:
                    reason = "x1<x1_lower_lim"
                elif constraint == 2:
                    reason = "x1>x1_upper_lim"                
                elif constraint == 3:
                    reason = "x2<x2_lower_lim"                     
                elif constraint ==4:
                    reason = "L>U"
                elif constraint > 4:
                    reason = "additional constraint violated" 
            else:
                trajectory.append(state)
            i=i+1

        return trajectory, u_list, L_list, A_list, D_list, A_list_with_state, D_list_with_state, u_list_with_state, L_list_with_state, reason        

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
            A, D = self.calc_upper_lower_bound_values(state)
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

    def check_if_constraints_violated_analytically(self,\
                                                   state):
        """
        Inputs
            state: state to test [x1, x2]
            constraints: 
        [   -x1 + x1_lower_lim, 
            x1 - x1_upper_lim, 
            -x2 + x2_lower_lim,
            D(x) - A(x), 
            C(x) 
        ]
            
                generally
                x1_lower_lim will be a float (usually 0)
                x1_upper_lim will be a float (usually 1)
                x2_lower_lim will be a float (usually 0)
                A(x)-D(x) will be an expression ()
                C(x) will be an expression for the other constraints
            
        """
        x1, x2, Ax, Dx, Cx\
        = symbols('x1 x2 Ax Dx Cx')
        
        state_sub_list = [(x1, state[0]), (x2, state[1])]
        constraints = self.constraint_list
        no_of_constraints_violated = 0
        i = 1
        for constraint in constraints:
            #print("constraint to check", i, constraint)
            if i == 4:
                A, D = self.calc_upper_lower_bound_values(state)
                limits_sub_list = [(Ax, A), (Dx, D)]
                #print("A, D", state, (A, D))
                constraint_value = constraint.subs(limits_sub_list) 
            else:
                constraint_value =  constraint.subs(state_sub_list) 
            #print("constraint_value", constraint_value)
            if constraint_value > 0:
                no_of_constraints_violated = 1
                break
            #print("constraint value ", i, " ", constraint_value)
            i = i + 1
        
        #print("number of constraints violated = ", no_of_constraints_violated )
        if no_of_constraints_violated > 0:
            return True, i
        else:
            return False, -1
    def set_constraint_list(self, constraint_list):
        
        """
        This method will allow the entire list of constraints to be directly
        swapped such that more can be added
         Parameters
        ----------
        constraint_list : TYPE
            list of constraints in the form of sympy equations
        """
        #print("constraint_list updated", constraint_list)
        
        self.constraint_list =  constraint_list
        
    def min_of_constraints(self):
        """
        return the minumum of the constraints
        """
        
        #print(self.constraint_list)
        
        constraint_list = self.constraint_list
        x1_upper_lim = self.x1_lims[1]
        x1_lower_lim = self.x1_lims[0]
        x1_step_size = self.x1_lims[2]
        
        #x1_list = np.arange(x1_lower_lim, x1_upper_lim, x1_step_size)        
        x1_list = np.linspace(0.0, 1.0, 1000)   

        min_constraint_curve = []
        x2_upper_bound = self.x2_lims[1]
        x2_lower_bound = self.x2_lims[0]
        x2u = x2_upper_bound
        x2l = x2_lower_bound

        constraint_violated = False
        epsilon= 0.1
        
        #outer loop checks along each x_1 point
        for x1 in x1_list:
            
            constraint_boundary_found=False
            x2u = x2_upper_bound
            x2l = x2_lower_bound
            
            x2 = (x2u +  x2l)/2
            
            loop_counter=0
            
            #do binary search 
            while constraint_boundary_found == False and loop_counter < 1000:
                #print("current x2 = ",x2)
                constraint_violated=False
                #print("x1 - ", x1)
                #check each constraint in the constraint list until one breaks the loop
            
                state = (x1, x2)
                violated, _ = self.check_if_constraints_violated_analytically(state)
                if violated == True:
                    constraint_violated = True
                    x2u = x2
                else:
                    x2l = x2
                
                #print("x2 upper=", x2u, "x2 lower=", x2l)
                prev_x2 = x2
                x2 = (x2u +  x2l)/2
                
                if (abs(prev_x2 - x2) < epsilon and constraint_violated==False) or x2 < 0.001:
                    #print("boundary state added = ", (x1, x2))
                    min_constraint_curve.append((x1, x2))
                    constraint_boundary_found=True
                
                loop_counter = loop_counter + 1


        #print("min constraint curve", min_constraint_curve)
        #min_constraint_curve = [(1,1)]
        return min_constraint_curve
    
        
    def convex_controller_trajectory(self, start_state, ROS_lower_bounds, \
                                     ROS_upper_bounds, \
                                     epsilon = 0.001, \
                                     endline=1, \
                                     control_strategy=1):
        """
        Takes in the upper and lower ROS bounds as pre ordered list in terms of
        x1
        
        takes a start state and drives it to the goal using the ROS bounds
        
        """
        
        lower_bound = ROS_lower_bounds        
        upper_bound = ROS_upper_bounds
        state = start_state
        
        """
        define control stategies
        """
        #L = ["1", lower_bound, upper_bound]
        #L = [str(control_strategy)]
        L = control_strategy
        T, u_list, L_list, A_list, D_list, A_list_with_state,\
        D_list_with_state,  u_list_with_state, \
        L_list_with_state,   reason = \
        self.simulate_trajectory_with_control(state, L, direction=1, T=epsilon)
                        
        #print(T)
        control_trajectories = [T]
        L_values = L_list
        u_values = u_list
        
        return control_trajectories, L_values, u_values, u_list_with_state, L_list_with_state, A_list, D_list, A_list_with_state, D_list_with_state
        #repeat the process until

    def x1_binary_search(self, target_x1, list_to_search):
        """
        method to search through a list of states and return the index of the
        state closest to the target
        """
        
        upper_index = len(list_to_search) - 1
        lower_index = 0
        closest_point_found = False
        i=0
        prev_mid_range = -1
        while closest_point_found == False:
            
            mid_range = int((upper_index + lower_index)/2)
            
            #print("mid range ", mid_range)
            if target_x1 > list_to_search[mid_range][0]:
                lower_index = mid_range
            else:            
                upper_index = mid_range    
            
            if prev_mid_range == mid_range:
                closest_point_found = True
                
            prev_mid_range = mid_range
        
        return mid_range
        
    def lambda_raw_linear_interpolation(self, x2, x2_upper, x2_lower):
    
        return (x2 - x2_upper)/(x2_lower - x2_upper)

    
    def sigmoid(self, x):
        
        sig = 1/( 1 + exp( -10*(x-0.5) ) )
        return sig

    def saturation(self, x):
        
        if x >1:
            x=1
        elif x<0:
            x=0     
        return x


    def lambda_simple_linear_saturation(self, state):
        
        """
        form 2 straight lines for a specific case and apply to see what happens
        """
        x1 = state[0]
        x2 = state[1]
        
        """
        define the upper line
        """
        m1 = 2.5
        x2_upper = m1*x1 + 2
        """
        define the lower line
        """
        m2 = 1
        x2_lower = m2*x1 + 1

        linear_interpolation =  self.lambda_raw_linear_interpolation(x2, x2_upper, x2_lower)
        
        L = self.saturation(linear_interpolation)
        
        print("linear interpolation = ", state, L)

        return L
    
    def lambda_simple_quadratic_saturation(self, state):
            
        """
        form 2 straight lines for a specific case and apply to see what happens
        """
        x1 = state[0]
        x2 = state[1]
        
        """
        define the upper curve
        """
        x2_upper = 3*x1**2 + 3
        """
        define the lower curve
        """
        x2_lower = 3.5*x1**2 + 0.5
        
        linear_interpolation =  self.lambda_raw_linear_interpolation(x2, x2_upper, x2_lower)
        
        L = self.saturation(linear_interpolation)
        
        #if state[0] > 0.22 and state[0] < 0.29:
            #print("state and L", state, L)
        
        #print("qudratic guides = ", state, L)

        return L
        
    def lambda_single_guide_saturation(self, state):
            
        """
        form 2 straight lines for a specific case and apply to see what happens
        """
        x1 = state[0]
        x2 = state[1]
        
        """
        define the curve
        """
        x2_bound = 3*x1 + 0.5

        if x2 >= x2_bound:
            L=0
        else:
            L=1

        return L
    
    def lambda_single_guide_2_saturation(self, state):
            
        """
        form 2 straight lines for a specific case and apply to see what happens
        """
        x1 = state[0]
        x2 = state[1]
        
        """
        define the curve
        """
        #x2_bound = 3*x1 + 0.5
        x2_bound = 1.5625*(x1 - 0.1)**2 + 2
        if x2 >= x2_bound:
            L=0
        else:
            L=1
        print("L=", L, state)
        return L    
    

    def compute_torques_required(self, inputs_with_states):
        #print("hi", inputs_with_states)
        """
        u = [x[0] for x in inputs_with_states]
        states = [x[1] for x in inputs_with_states]
        
        torque_vector, tau_1_val_with_state,\
         tau_2_val_with_state, torque_vector_with_state\
             = self.manipulator.evaluate_joint_torques(u, states)
        """
        
        print("not implemented with matlab yet")
        
        torque_vector = [0]
        tau_1_val_with_state = [0]
        tau_2_val_with_state = [0]
        torque_vector_with_state = [0]
        
        return  torque_vector, tau_1_val_with_state, tau_2_val_with_state, torque_vector_with_state
    

    def lambda_different_guide_saturation(self, state):

        x1 = state[0]
        x2 = state[1]
        
        m1 = 4         
        m2 = 2
        
        """
        define the upper curve
        """
        x2_upper = -m1*(x1 - 0.6)**2 +4
        """
        define the lower curve
        """
        x2_lower = m2*(1.1*x1 - 0.5)**3 +2
        
        linear_interpolation =  self.lambda_raw_linear_interpolation(x2, x2_upper, x2_lower)
        
        L = self.saturation(linear_interpolation)
        
        #if state[0] > 0.22 and state[0] < 0.29:
            #print("state and L", state, L)
        
        #print("qudratic guides = ", state, L)

        return L
    
    def get_control_guides(self, control_type, number_steps=100):
        """
        Parameters
        ----------
        control_type : TYPE
            DESCRIPTION.
        number_steps : TYPE, optional
            DESCRIPTION. The default is 100.

        Returns
        -------
        None.

        """

        if control_type == "5":

            x1_list = np.linspace(0.0, 0.9, num=number_steps)

            lower_bound = []
            for x1 in x1_list:

                x2_upper = 1.5625*(x1 - 0.1)**2 + 2
                x2_lower = 1.5625*(x1 - 0.1)**2 + 2
                
                if len(lower_bound) == 0:
                    lower_bound = [(x1, x2_lower)]
                    upper_bound = [(x1, x2_upper)]
                else:
                    lower_bound.append((x1, x2_lower))
                    upper_bound.append((x1, x2_upper))
        
        
        elif control_type == "4":

            x1_list = np.linspace(0.0, 0.9, num=number_steps)

            lower_bound = []
            for x1 in x1_list:

                x2_upper = 3*x1 + 0.5
                x2_lower = 3*x1 + 0.5
                
                if len(lower_bound) == 0:
                    lower_bound = [(x1, x2_lower)]
                    upper_bound = [(x1, x2_upper)]
                else:
                    lower_bound.append((x1, x2_lower))
                    upper_bound.append((x1, x2_upper))
        
        
        elif control_type == "6":
            x1_list = np.linspace(0.0, 0.9, num=number_steps)
            m1 = 4         
            m2 = 2
            lower_bound = []
            for x1 in x1_list:
                
                x2_upper = -m1*(x1 - 0.6)**2 +4
                x2_lower = m2*(1.1*x1 - 0.5)**3 +2
                
                if len(lower_bound) == 0:
                    lower_bound = [(x1, x2_lower)]
                    upper_bound = [(x1, x2_upper)]
                else:
                    lower_bound.append((x1, x2_lower))
                    upper_bound.append((x1, x2_upper))

        
        bounds = [lower_bound, upper_bound]
        return bounds
            