# -*- coding: utf-8 -*-
"""
Created on Sun Mar 22 11:49:26 2020
This function will take a manipulator model and work out control actions

@author: Ryan
"""

from math import sqrt, exp, isnan
import numpy as np
import sympy
import my_math as mm
import matplotlib.pyplot as plt
from sympy import symbols, sin, cos
from my_sorting import combine_to_tuples
import time

from sympy.utilities.autowrap import ufuncify

class path_dynamics_controller():
    """
    This class will contain the methods for designing a controller for moving
    the robot along the state space.
    """
    def __init__(self, manipulator, form_sped_up_constraints=True):
        #print("initialised")
        self.manipulator = manipulator
        self.bounds = manipulator.bounds
        self.joint_limits = manipulator.joint_limits
        self.boundary_points = manipulator.boundary_points
        self.s = manipulator.s
        self.sd = manipulator.sd
        self.constraint_list = self.form_constraint_list()
        self.x1_lims = manipulator.s_lims
        self.x2_lims = manipulator.sd_lims
        self.x1_sym, self.x2_sym\
        = symbols('x1 x2')
        

        self.constriant_violation_condition = {
            "0":"x1<x1_lower_lim",
            "1":"x1>x1_upper_lim" ,            
            "2":"x2<x2_lower_lim",
            "3":"L>U",
            ">3":"additional constraint violated" 
        }

        print("Initialising fast evaluation funtions")
        self.build_lamdifyed_constraint_matrix_for_fast_evaluation()
        self.build_lamdifyed_upper_lower_bound_for_fast_evaluation()

        if form_sped_up_constraints == True:
            self.build_ufuncify_constraint_matrix_for_fast_evaluation()
            self.build_ufuncify_upper_lower_bound_for_fast_evaluation()
        else:
            #self.fast_constraint_matrix = ""
            self.very_fast_Ax_Dx_list = ""

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

    def build_ufuncify_upper_lower_bound_for_fast_evaluation(self):
        constraints = self.constraint_list
        x1, x2\
        = symbols('s sd')
        #sympy_matrix = sympy.Matrix(self.bounds)
        numpy_bounds = np.array(self.bounds)
        #very_fast_Ax_Dx_list = [ufuncify([x1, x2], el, backend='cython',flags = ['-D_USE_MATH_DEFINES']) for row in numpy_bounds for el in row]#unfuncity all elements in the list self.bounds
        very_fast_Ax_Dx_list = [ufuncify([x1, x2], el, backend='cython') for row in numpy_bounds for el in row]#unfuncity all elements in the list self.bounds
        #print("Done!")
        numpy_array_version = np.array(very_fast_Ax_Dx_list)#convert to numpy array
        a = numpy_array_version.reshape(int(len(very_fast_Ax_Dx_list)/3), 3)#reshape to original shape
        unfuncify_functions = np.delete(a, 2, axis=1)#cut off third column 

        unfuncify_functions = unfuncify_functions.flatten() #flatten them to evaluate faster latere
        self.very_fast_Ax_Dx_list = unfuncify_functions


    def fast_evaluate_ufuncify_functions_upper_lower_bound_analytically(self, state, very_fast_Ax_Dx_list):
        """
        Function that uses unfuncify to quickly evaluate and check matrices
        """
        result_array = [el(np.array([state[0]], dtype=np.double),np.array([state[1]], dtype=np.double)) for el in very_fast_Ax_Dx_list]
        result_array = np.array(result_array)
        result = result_array.reshape(int(len(result_array)/2), 2)#reshape to two column matrix
        U = float(np.min(np.max(result, axis=1)))
        L = float(np.max(np.min(result, axis=1)))

        return L, U

    def fast_evaluate_upper_lower_bound_analytically(self, state):
        """
        Function that uses numpy to quickly evaluate and check matrices
        """

        result = self.fast_Ax_Dx_matrix(float(state[0]), float(state[1]))

        rows, _ = result.shape
        val = result[0:rows, 0:2]
        U = float(np.min(np.max(val, axis=1)))
        L = float(np.max(np.min(val, axis=1)))

        return L, U

    def build_lamdifyed_upper_lower_bound_for_fast_evaluation(self):
        constraints = self.constraint_list
        x1, x2\
        = symbols('s sd')
        sympy_matrix = sympy.Matrix(self.bounds)

        self.fast_Ax_Dx_matrix= sympy.lambdify([x1, x2], sympy_matrix, "numpy")

    def fast_evaluate_upper_lower_bound_analytically(self, state):
        """
        Function that uses numpy to quickly evaluate and check matrices
        """
        result = self.fast_Ax_Dx_matrix(float(state[0]), float(state[1]))

        rows, _ = result.shape
        val = result[0:rows, 0:2]
        U = float(np.min(np.max(val, axis=1)))
        L = float(np.max(np.min(val, axis=1)))

        return L, U

    # def integrate(self, pos, step_size, polynomial_coefficents, acceleration_choice, direction):
    #     """
    #     Arguments:
    #         pos - (s, sd) tuple of numbers
    #         step_size - float that decides how big each integration step should be.
    #         acceleration_choice - this is a string "U" will give max possible acceleration
    #                                                "L" will give min possible acceleration
    #                               add functionality for other accelerations
    #         direction - string giving the direction of the integration
    #     return:
    #         trajectory - [(s1,sd1), ... , (sn, sdn)]
    #         violations-                 
    #             if sd> np.polyval(polynomial_coefficents, s):
    #                 violation = 0 #"Safety_Constraint_Volation"
    #             elif s < 0:
    #                 violation = 1 #"s<0"
    #             elif s > 1:
    #                 violation = 2 #"s>1"
    #             elif sd < 0:
    #                 violation = 3 #"sd<0"
    #     """

    #     trajectory = [pos]
    #     boundries = []
    #     i = 0
        
    #     s = pos[0]
    #     sd = pos[1]
    
    #     integration_complete = False
    #     violation = "integration not completed yet"
        
    #     while(integration_complete == False and i<1000):
            
    #         #calculate -L
    #         #print(current_pos)
    #         L, U = self.calc_upper_lower_bound_values(pos)
    #         s = pos[0]
    #         sd = pos[1]
            
    #         if i == 0:
    #             boundries = [(L, U)]
    #         else:
    #             boundries.append((L, U))            


    #         if acceleration_choice == "U":
    #             up_down_acc = U
    #         elif acceleration_choice == "L":
    #              up_down_acc = L 
            
    #         #print(L, U)
    #         if direction == "backwards":
    #             delta_s, delta_sd = self.calc_change_s_sd(-sd, -up_down_acc, step_size)#integrate backwards
    #         elif direction == "forwards":
    #             delta_s, delta_sd = self.calc_change_s_sd(sd, up_down_acc, step_size)#integrate forwards

    #         s = s + delta_s
    #         sd = sd + delta_sd
    #         pos = (s, sd)
    #         #print(pos)
    #         #print(s, sd, np.polyval(polynomial_coefficents, s))
            
    #         if sd > np.polyval(polynomial_coefficents, s) or s < 0 or s > 1 or sd < 0:
    #             integration_complete = True
    #             if sd> np.polyval(polynomial_coefficents, s):
    #                 violation = 0#"Safety_Constraint_Volation"
    #             elif s < 0:
    #                 violation = 1#"s<0"
    #             elif s > 1:
    #                 violation = 2#"s>1"
    #             elif sd < 0:
    #                 violation = 3#"sd<0"
   
    #         else:
    #             trajectory.append(pos)                
    #         #print(trajectory)
    #         i=i+1
            
    #     if direction == "backwards":#reverse backwards trajectories
    #          trajectory.reverse()
            
    #     return violation, trajectory

    # def simulate_trajectory(self, X0, L, direction=1, T=0.01, x1_lim=1):
    #     """
    #     Parameters
    #     ----------
    #     X0 : (x_1, x_2)
    #         beginning state
    #     T : float
    #         sample size for simulation steps
    #     L : TYPE
    #         L the npercentage of actuation between U and L, value 0 < L < 1
    #     direction : +1 or -1, optional
    #         DESCRIPTION. The default is 1 representing forward integration

    #     Returns
    #     -------
    #     trajectory
    #     """
        
    #     x2_max = self.x2_lims[1]
        
    #     #print(X0)
    #     #change default x1_lim if direction is reversed
    #     if direction==-1 and x1_lim ==1:
    #         x1_lim = 0
        
    #     state = X0
    #     i=0
    #     trajectory = [state]
    #     boundries = []
             
    #     integration_complete = False
    #     reason = "integration not completed yet"
        
    #     while(integration_complete == False and i<10000):
    #         #print("i = ", i)
    #         #determine the max and min range values for the input
    #         D, A = self.calc_upper_lower_bound_values(state)
    #         #print((i/5000)*100, " % complete", (A, D))
    #         x1 = state[0]
    #         x2 = state[1]
            
    #         if i == 0:
    #             boundries = [(D, A)]
    #         else:
    #             boundries.append((D, A))            
            
    #         if D <= A:
    #             #choose the input based on the actuation level L
    #             u = D + L*(A-D)
    #             try:
    #                 delta_x1, delta_x2 = self.calc_change_s_sd(direction*x2, direction*u, T)
    #             except: 
    #                 print("I gotcha")
    #             x1 = x1 + delta_x1
    #             x2 = x2 + delta_x2
    #             state = (x1, x2)
    #             #print("state: ", state)
    #             violated, constraint =\
    #                 self.check_if_constraints_violated_analytically(state) 
                
    #             if violated == True:
    #                 integration_complete = True
    #                 if constraint == 1:
    #                     reason = "x1<x1_lower_lim"
    #                 elif constraint == 2:
    #                     reason = "x1>x1_upper_lim"                
    #                 elif constraint == 3:
    #                     reason = "x2<x2_lower_lim"                     
    #                 elif constraint ==4:
    #                     reason = "L>U"
    #                 elif constraint > 4:
    #                     reason = "additional constraint violated" 
    #             else:
    #                 trajectory.append(state)
    #         else:
    #             integration_complete = True
                
    #         i=i+1

    #     return trajectory, boundries, reason        
    
    def simulate_trajectory_with_control(self, X0, direction=1, T=0.01, data_log=False):
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
        self.integration_time = 0 #total tiem for the whole while loop to run
        self.total_time_choosing_A_D = 0 #total time spent choosing our limits A and D
        self.total_time_taken_getting_L_values = 0 #variable for storing the time to calculate relevant L values
        self.total_time_appending_values = 0 ##variable for storing the time to calculate relevant appending values
        self.total_time_calculating_move = 0 ##variable for storing the time to calculate relevant appending values
        self.total_time_checking_constraints = 0 #total time spent checking the constraints
        self.total_time_adding_constraints_message = 0#total time taken to add a reason to the constraint violation
        self.total_time_running_each_loop = 0#time from top to bottom of each loop

        state = X0
        i=0
        trajectory = [state]
        actuation_function = self.Lambda_x
        very_fast_Ax_Dx_list = self.very_fast_Ax_Dx_list
        boundries = []
        u_list = []
        L_list = []
        A_list = [] 
        D_list = []
        u_list_with_state = []
        L_list_with_state = []
        A_list_with_state = [] 
        D_list_with_state = []


        start_integration_time =  time.time()
        while True: #integration_complete == False):

            start_loop_time = time.time()
            #determine the max and min range values for the input
            #D, A = self.fast_evaluate_upper_lower_bound_analytically(state)
            try:
                D, A = self.fast_evaluate_ufuncify_functions_upper_lower_bound_analytically(state, very_fast_Ax_Dx_list)
                if isnan(D):
                    altD, _ = self.calc_upper_lower_bound_values(state)
                    D= altD

                if isnan(A):
                    _, altA = self.calc_upper_lower_bound_values(state)
                    A = altA
            except:
                print("Something wrong with the evaluation of A and D")
                break
            
            end_choose_limits = time.time()
            self.total_time_choosing_A_D = self.total_time_choosing_A_D + (end_choose_limits - start_loop_time)


            """
            ================input control=============================
            """
            start_choose_L = time.time()
            L = actuation_function(state)
            
            end_choose_L = time.time()
            self.total_time_taken_getting_L_values = self.total_time_taken_getting_L_values + (end_choose_L - start_choose_L)
            """
            ================================================================
            """
            u = D + L*(A-D)

            start_appends = time.time()
            if data_log:
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

            end_appends= time.time()

            self.total_time_appending_values = self.total_time_appending_values + (end_appends - start_appends)
            
            start_calc_move = time.time()

            delta_x1, delta_x2 = self.calc_change_s_sd(float(direction*state[1]), float(direction*u), T)

            x1 = state[0] + delta_x1
            x2 = state[1] + delta_x2
            state = (x1, x2)

            end_calc_move = time.time()
            self.total_time_calculating_move = self.total_time_calculating_move + (end_calc_move - start_calc_move)


            #====================================Check constraints==================================================
            start_check_constraints = time.time()
            violated, constraint =self.fast_evaluate_constraints_analytically(state, A, D)
            #violated, constraint =self.fast_evaluate_ufuncify_constraint_matrix(state, A, D)
            end_check_constraints = time.time()
            self.total_time_checking_constraints = self.total_time_checking_constraints + (end_check_constraints - start_check_constraints)

            start_add_constraint_message = time.time()  
            if violated == True:
                if constraint < 4:
                    constraint_key = str(constraint)
                else:
                    constraint_key = ">3"
                reason = self.constriant_violation_condition[constraint_key]
                break
            else:
                trajectory.append(state)

            end_add_constraint_message = time.time()
            self.total_time_adding_constraints_message = self.total_time_adding_constraints_message + (end_add_constraint_message - start_add_constraint_message)
            
            i=i+1
            end_loop_time = time.time()
            self.total_time_running_each_loop = self.total_time_running_each_loop + (end_loop_time - start_loop_time)

        end_integration_time =  time.time()
        self.integration_time = end_integration_time - start_integration_time
        return trajectory, u_list, L_list, A_list, D_list, A_list_with_state, D_list_with_state, u_list_with_state, L_list_with_state, reason        
    
    # def fast_simulate_trajectory_with_control(self, X0, direction=1, T=0.01, log_data=False):
    #     """
    #     Parameters
    #     ----------
    #     X0 : (x_1, x_2)
    #         beginning state
    #     T : float
    #         sample size for simulation steps
    #     self.Lambda_x : Function that sets actuation level
    #         Lambda_x the percentage of actuation between U and L, value 0 < L < 1
    #     direction : +1 or -1, optional
    #         DESCRIPTION. The default is 1 representing forward integration
    #     Returns
    #     -------
    #     trajectory
    #     """
    #     self.integration_time = 0 #total tiem for the whole while loop to run
    #     self.total_time_choosing_A_D = 0 #total time spent choosing our limits A and D
    #     self.total_time_taken_getting_L_values = 0 #variable for storing the time to calculate relevant L values
    #     self.total_time_appending_values = 0 #variable for storing the time to calculate relevant appending values
    #     self.total_time_calculating_move = 0 #variable for storing the time to calculate relevant appending values
    #     self.total_time_checking_constraints = 0 #total time spent checking the constraints
    #     self.total_time_adding_constraints_message = 0 #total time taken to add a reason to the constraint violation
    #     self.total_time_running_each_loop = 0 #time from top to bottom of each loop

    #     state = X0
    #     i=0
    #     trajectory = [state]
        
    #     boundries = []
    #     u_list = []
    #     L_list = []
    #     A_list = [] 
    #     D_list = []
    #     u_list_with_state = []
    #     L_list_with_state = []
    #     A_list_with_state = [] 
    #     D_list_with_state = []

    #     integration_complete = False
    #     reason = "integration not completed yet"
                
    #     start_integration_time =  time.time()
    #     while(integration_complete == False):

    #         start_loop_time = time.time()
    #         """
    #         ==============================Set values of A and D====================================
    #         """
    #         #determine the max and min range values for the input                
    #         #D, A = self.fast_evaluate_upper_lower_bound_analytically(state)
    #         try:
    #             D, A = self.fast_evaluate_ufuncify_functions_upper_lower_bound_analytically(state)
    #             if isnan(D):
    #                 altD, altA = self.calc_upper_lower_bound_values(state)
    #                 D= altD
    #             if isnan(A):
    #                 altD, altA = self.calc_upper_lower_bound_values(state)
    #                 A = altA
    #         except:
    #             print("Something wrong with the evaluation of A and D")
    #             break
    #         x1, x2 = state[0], state[1]
    #         if i == 0:
    #             boundries = [(D, A)]
    #         else:
    #             boundries.append((D, A))    

    #         end_choose_limits = time.time()
    #         self.total_time_choosing_A_D = self.total_time_choosing_A_D + (end_choose_limits - start_loop_time)
    #         """
    #         ================input control=============================
    #         """
    #         start_choose_L = time.time()
    #         L = self.Lambda_x(state)
    #         end_choose_L = time.time()
    #         self.total_time_taken_getting_L_values = self.total_time_taken_getting_L_values + (end_choose_L - start_choose_L)

    #         u = D + L*(A-D)
    #         start_appends = time.time()
    #         if len(u_list)== 0 and log_data == True:
    #             u_list = [u]
    #             L_list = [L]
    #             A_list = [A] 
    #             D_list = [D]
    #             u_list_with_state = [(u, state)]
    #             L_list_with_state = [(L, state)]
    #             A_list_with_state = [(A, state)] 
    #             D_list_with_state = [(D, state)]
    #         else:
    #             u_list.append(u)
    #             L_list.append(L)
    #             A_list.append(A)
    #             D_list.append(D)
    #             u_list_with_state.append((u, state))
    #             L_list_with_state.append((L, state))
    #             A_list_with_state.append((A, state))
    #             D_list_with_state.append((D, state))
            
    #         end_appends= time.time()
    #         self.total_time_appending_values = self.total_time_appending_values + (end_appends - start_appends)
            
    #         start_calc_move = time.time()
            
    #         delta_x1, delta_x2 = self.calc_change_s_sd(float(direction*x2), float(direction*u), T)
            
    #         x1 = x1 + delta_x1
    #         x2 = x2 + delta_x2

    #         state = (x1, x2)

    #         end_calc_move = time.time()
    #         self.total_time_calculating_move = self.total_time_calculating_move + (end_calc_move - start_calc_move)

    #         start_check_constraints = time.time()

    #         result = self.fast_constraint_matrix(float(state[0]), float(state[1]), float(A), float(D))
    #         if (result> 0).sum()>0:
    #             violated= True, 
    #             constraint = (result> 0).sum()
    #         else:
    #             violated= False, 
    #             constraint = -1

    #         end_check_constraints = time.time()
    #         self.total_time_checking_constraints = self.total_time_checking_constraints + (end_check_constraints - start_check_constraints)
            
    #         start_add_constraint_message = time.time()

    #         if violated == True:
    #             integration_complete = True
    #             if constraint == 1:
    #                 reason = "x1<x1_lower_lim"
    #             elif constraint == 2:
    #                 reason = "x1>x1_upper_lim"                
    #             elif constraint == 3:
    #                 reason = "x2<x2_lower_lim"                     
    #             elif constraint ==4:
    #                 reason = "L>U"
    #             elif constraint > 4:
    #                 reason = "additional constraint violated"  
    #         else:
    #             trajectory.append(state)

    #         end_add_constraint_message = time.time() 
    #         self.total_time_adding_constraints_message = self.total_time_adding_constraints_message + (end_add_constraint_message - start_add_constraint_message)
            
    #         i=i+1
    #         end_loop_time = time.time()
    #         self.total_time_running_each_loop = self.total_time_running_each_loop + (end_loop_time - start_loop_time)

    #     end_integration_time =  time.time()

    #     self.integration_time = end_integration_time - start_integration_time

    #     return trajectory, u_list, L_list, A_list, D_list, A_list_with_state, D_list_with_state, u_list_with_state, L_list_with_state, reason        

    def select_lambda_x(self, L):
        """
        directly use the passed in L value
        """
        self.control_function_info = L

        if isinstance(L, float):
            self.constant_actuation_level= L
            Lambda_x = self.raw_control

        #elif apply some form of control
        elif isinstance(L, list):
            control_strategy =L[0]
            if control_strategy=="linear interpolation control":
                Lambda_x = self.raw_linear_interpolation_control

            elif control_strategy=="custom guides":
                L = self.control_function_info
                x1 = symbols('x1')
                sympy_matrix = sympy.Matrix([L[2], L[1]])
                self.fast_control_bounds= sympy.lambdify([x1], sympy_matrix, "numpy")

                Lambda_x = self.custom_linear_interpolation_control
        else:
            print("something is wrong with input L")
        
        self.Lambda_x = Lambda_x

    def raw_control(self, state):
        return self.control_function_info

    def raw_linear_interpolation_control(self, state):
        L = self.control_function_info
        x1 = state[0]
        x2 = state[1]

        lower_bound = L[1]
        upper_bound = L[2]
        
        close_index = self.x1_binary_search(x1, lower_bound)
        x2_lower = lower_bound[close_index][1]
        """
        find closest x1 point on the upper bound (binary search)
        """
        close_index = self.x1_binary_search(x1, upper_bound)
        x2_upper = upper_bound[close_index][1]
        actuation_level = self.lambda_raw_linear_interpolation(x2, x2_upper, x2_lower)
        return actuation_level

    def custom_linear_interpolation_control(self, state):

        x2_upper_and_lower = self.fast_control_bounds(state[0])
        x2_lower = x2_upper_and_lower[1] #lower_bound.subs(self.x1_sym, x1)
        x2_upper = x2_upper_and_lower[0] #upper_bound.subs(self.x1_sym, x1)
        actuation_level = self.lambda_raw_linear_interpolation(state[1], x2_upper, x2_lower)

        return actuation_level

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
            A(x)-D(x), 
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
        #print("constraints considered are: " , constraints)
        for constraint in constraints:
            if i == 4:
                D, A = self.calc_upper_lower_bound_values(state)
                limits_sub_list = [(Ax, A), (Dx, D)]
                
                constraint_value = constraint.subs(limits_sub_list) 
            else:
                constraint_value =  constraint.subs(state_sub_list) 
            
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

    def build_lamdifyed_constraint_matrix_for_fast_evaluation(self):
        constraints = self.constraint_list
        x1, x2, Ax, Dx\
        = symbols('x1 x2 Ax Dx')
        sympy_matrix = sympy.Matrix(constraints)
        self.fast_constraint_matrix= sympy.lambdify([x1, x2, Ax, Dx], sympy_matrix, "numpy")

    def build_ufuncify_constraint_matrix_for_fast_evaluation(self):
        x1, x2, Ax, Dx\
        = symbols('x1 x2 Ax Dx')

        numpy_constraints = self.constraint_list#np.array(self.constraint_list[0])

        very_fast_constraint_list = [ufuncify([x1, x2, Ax, Dx], el, backend='cython',flags = ['-D_USE_MATH_DEFINES']) for el in numpy_constraints]
        self.very_fast_constraint_list = very_fast_constraint_list

    def fast_evaluate_ufuncify_constraint_matrix(self, state, A, D):
        """
        Function that uses numpy to quickly evaluate and check matrices
        """
        result_array = [el(np.array([state[0]], dtype=np.double), np.array([state[1]], dtype=np.double)\
                        ,np.array([A], dtype=np.double), np.array([D], dtype=np.double))\
                        for el in self.very_fast_constraint_list]
        result_array = np.array(result_array)

        if (result_array> 0).sum()>0:
            return True, (result_array > 0).sum()
        else:
            return False, -1

    def very_fast_evaluate_constraints_analytically(self, state, A, D):
        """
        Function that uses numpy to quickly evaluate and check matrices
        """

        result = self.fast_constraint_matrix(float(state[0]), float(state[1]), float(A), float(D))

        if (result> 0).sum()>0:
            return True, np.argwhere(result> 0)
        else:
            return False, -1


    def fast_evaluate_constraints_analytically(self, state, A, D):
        """
        Function that uses numpy to quickly evaluate and check matrices
        """
        
        result = self.fast_constraint_matrix(float(state[0]), float(state[1]), float(A), float(D))

        if (result> 0).sum()>0:
            return True, int(np.argwhere(result> 0)[0,0])
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
        self.constraint_list =  constraint_list
        
        self.build_lamdifyed_constraint_matrix_for_fast_evaluation()#rebuild the fast version
        
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
        x1_list = np.linspace(0, 1, 1000)   

        min_constraint_curve = []
        x2_upper_bound = self.x2_lims[1]
        x2_lower_bound = self.x2_lims[0]
        x2u = x2_upper_bound
        x2l = x2_lower_bound

        constraint_violated = False
        epsilon= 0.1
        
                
        for x1 in x1_list:
            constraint_boundary_found=False
            x2u = x2_upper_bound
            x2l = x2_lower_bound
            x2 = (x2u +  x2l)/2
            loop_counter=0
            while constraint_boundary_found == False and loop_counter < 1000:
                #print("current x2 = ",x2)
                constraint_violated=False
                for constraint in constraint_list:
                    state = (x1, x2)
                    violated, _ = self.check_if_constraints_violated_analytically(state)
                    if violated == True:
                        constraint_violated = True
                        break
                
                if constraint_violated == True:
                    x2u = x2
                else:
                    x2l = x2
                
                #print("x2 upper=", x2u, "x2 lower=", x2l)
                prev_x2 = x2
                x2 = (x2u +  x2l)/2
                
                if abs(prev_x2 - x2) < epsilon and constraint_violated==False:
                    #print("boundary state added = ", (x1, x2))
                    min_constraint_curve.append((x1, x2))
                    constraint_boundary_found=True
                loop_counter = loop_counter + 1


        #print("min constraint curve", min_constraint_curve)
        #min_constraint_curve = [(1,1)]
        return min_constraint_curve
    
    def convex_controller_trajectory(self, start_state, \
                                     epsilon = 0.001, endline=1):
        """
        Takes in the upper and lower ROS bounds as pre ordered list in terms of
        x1
        
        takes a start state and drives it to the goal using the ROS bounds
        """
        

        state = start_state
  
        T, u_list, L_list, A_list, D_list, A_list_with_state,\
        D_list_with_state,  u_list_with_state, \
        L_list_with_state,   reason = \
        self.simulate_trajectory_with_control(state, direction=1, T=epsilon)
                        
        #print(T)
        control_trajectories = [T]
        L_values = L_list
        u_values = u_list
        
        return control_trajectories, L_values, u_values, u_list_with_state, L_list_with_state, A_list, D_list, A_list_with_state, D_list_with_state
        #repeat the process until


        
    def old_convex_controller_trajectory(self, start_state, ROS_lower_bounds, \
                                     ROS_upper_bounds, \
                                     epsilon = 0.01, endline=1, control_strategy=1):
        """
        Takes in the upper and lower ROS bounds as pre ordered list in terms of
        x1
        
        takes a start state and drives it to the goal using the ROS bounds
        
        """
        
        lower_bound = ROS_lower_bounds        
        upper_bound = ROS_upper_bounds
        state = start_state



        #print(lower_bound[1])
        #print(upper_bound[0])        
        x1= x1 = state[0]

        control_trajectories = []
        L_values = []
        end_not_reached = True
        while end_not_reached:
            
            x1 = state[0]
            x2 = state[1]
               
            #close_index = self.x1_binary_search(0.9, upper_bound)
            #print(upper_bound[close_index])
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
            
            """
            calculate the required actuation level L using bounds
            """
            if control_strategy==1:
                L = self.lambda_raw_linear_interpolation(x2, x2_upper, x2_lower)
            elif control_strategy==2:
                L = self.lambda_simple_linear_saturation(state)
            elif control_strategy==3:
                L = self.lambda_simple_quadratic_saturation(state)
            #print("L = ", L)
            #print("state = ", state )
            """
            modify the constraints so the trajectory will go a very short distance 
            in terms of the x1 this method stupidly requires 5 constraints,
            4 physical and one additional
            """
            cl = self.constraint_list
            self.set_constraint_list([ cl[0], \
                                        cl[2], \
                                        cl[2], \
                                        cl[3], \
                                        cl[4],\
                                        self.x1_sym-(x1+epsilon)] )
            """
            simulate a length of trajectory using this L
            """
            T, u_list, reason = self.simulate_trajectory(state, L, 1, T=epsilon/1.01)
            
            """
            set state for next loop
            """
            state = T[-1]
            #print(state, reason)
            if len(control_trajectories) == 0:
                control_trajectories = [T]
                L_values = [L]
                u_values = u_list
                L_val_with_state = [(L, state)]
                
            else:
                control_trajectories.append(T)
                L_values.append(L)
                u_values.extend(u_list)
                L_val_with_state.append((L, state))
            
            if state[0] > endline:#reason == "x1>x1_upper_lim":
                end_not_reached = False
            
        #print(T)

        return control_trajectories, L_values, u_values, L_val_with_state


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


    # def lambda_simple_linear_saturation(self, state):
        
    #     """
    #     form 2 straight lines for a specific case and apply to see what happens
    #     """
    #     x1 = state[0]
    #     x2 = state[1]
        
    #     """
    #     define the upper line
    #     """
    #     m1 = 2
    #     x2_upper = m1*x1 + 3
    #     """
    #     define the lower line
    #     """
    #     m2 = 3
    #     x2_lower = m2*x1 + 1

    #     linear_interpolation =  self.lambda_raw_linear_interpolation(x2, x2_upper, x2_lower)
        
    #     L = self.saturation(linear_interpolation)
    #     return L
    
    # def lambda_simple_quadratic_saturation(self, state):
            
    #     """
    #     form 2 straight lines for a specific case and apply to see what happens
    #     """
    #     x1 = state[0]
    #     x2 = state[1]
        
    #     """
    #     define the upper curve
    #     """
    #     x2_upper = 3*x1**2 + 3
    #     """
    #     define the lower curve
    #     """
    #     x2_lower = 3.5*x1**2 + 0.5
    #     linear_interpolation =  self.lambda_raw_linear_interpolation(x2, x2_upper, x2_lower)
    #     L = self.saturation(linear_interpolation)

    #     return L
    
    # def lambda_single_guide_saturation(self, state):   
    #     """
    #     form 2 straight lines for a specific case and apply to see what happens
    #     """
    #     x1 = state[0]
    #     x2 = state[1]
        
    #     """
    #     define the curve
    #     """
    #     x2_bound = 3*x1**2 + 3

    #     if x2 >= x2_bound:
    #         L=0
    #     else:
    #         L=1

    #     return L
    
    # def lambda_single_guide_2_saturation(self, state):
    #     """
    #     form 2 straight lines for a specific case and apply to see what happens
    #     """
    #     x1 = state[0]
    #     x2 = state[1]
        
    #     """
    #     define the curve
    #     """
    #     x2_bound = 3*x1 + 0.5

    #     if x2 >= x2_bound:
    #         L=0
    #     else:
    #         L=1
    #     print("L=", L, state)
    #     return L    
    
    
    def compute_torques_required(self, inputs_with_states):
        #print("hi", inputs_with_states)
        u = [x[0] for x in inputs_with_states]
        states = [x[1] for x in inputs_with_states]
        
        #print("st", states)
        
        torque_vector, tau_1_val_with_state,\
         tau_2_val_with_state, torque_vector_with_state\
             = self.manipulator.evaluate_joint_torques(u, states)
        return  torque_vector, tau_1_val_with_state, tau_2_val_with_state, torque_vector_with_state
    
    
    # def approximate_sx(self, constraint_data):
    #     """
    #     method calculates S(x) for each point then does a polynomial approximation of the result
    #     """
    #     pass
    

        
        
        
        