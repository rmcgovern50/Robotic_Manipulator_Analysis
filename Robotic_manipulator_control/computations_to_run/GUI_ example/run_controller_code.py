import sympy
from sympy import symbols

import functions.ROS_simulations_plotting as plotting
import functions.ROS_algorithm_simulations as sim
#import functions.ml_ROS_algorithm_simulations as sim

import datetime as dt

import numpy as np
from my_sorting import combine_to_tuples
from functions.controller import path_dynamics_controller


import save_data as save_data
from scipy.optimize import lsq_linear as lsq_linear

import json

import matplotlib.pyplot as plt
import lsqlin

import timeit
import gc

class controller_simulation:
    def __init__(self, simulation_object, GUI_input_data):
        self.data = GUI_input_data
        self.simulation_object = simulation_object
        self.control = simulation_object.control
        self.lower_bound_ROS = simulation_object.lower_bound_ROS
        self.upper_bound_ROS = simulation_object.upper_bound_ROS
        print("created the control class")
        
        self.current_time = dt.datetime.now()

    def run_state_feedback_controller(self, time_and_print =True):
        """
        Run the state feedback control strategy the GUI inputs configure the details
        """
        import time
        print("running controller function")
        start_time = time.time()
        data = self.data
        
        start_state = (data.x1_start_controller, data.x2_start_controller)
        max_step = data.max_step_size_controller
        ROS_lower_bounds = self.lower_bound_ROS
        ROS_upper_bounds = self.upper_bound_ROS        
        
        ROS_lower_bounds.reverse()
        ROS_upper_bounds.reverse()
        
        if self.data.upper_guide == "N/A" and self.data.lower_guide == "N/A":
            control_strategy_description = ["linear interpolation control",\
                                            ROS_lower_bounds,\
                                            ROS_upper_bounds]
        else:
            control_strategy_description = ["custom guides",\
                                            self.data.lower_guide,\
                                            self.data.upper_guide]

        self.control.select_lambda_x(control_strategy_description)

        control_trajectory,\
        L_values, u_values,\
        u_list_with_state,\
        L_list_with_state,\
        A_list, D_list,\
        A_list_with_state,\
        D_list_with_state \
            = self.control.convex_controller_trajectory(start_state, \
                                     epsilon = max_step, endline=1)
        
        
        print("controller run complete")

        self.control_trajectory = control_trajectory[0]
        end_time =  time.time()    
        seconds = end_time - start_time
        self.fb_control_runtime = seconds


        #msg = "The total runtime for the controller is "
        #self.print_time(msg, seconds)


        if time_and_print == True:
            msg = "The runtime for the integration is "
            time = self.control.integration_time
            self.print_time(msg, time)


            msg = " total_time_spent_choosing_actuator limits A and D "
            time= self.control.total_time_choosing_A_D
            self.print_time(msg, time)
            
            msg = " total_time_spent_choosing_actuation_levels "
            time= self.control.total_time_taken_getting_L_values
            self.print_time(msg, time)


            msg = " total time spent appending stuff "
            time= self.control.total_time_appending_values
            self.print_time(msg, time)

            msg = " total time spent calculating move "
            time= self.control.total_time_calculating_move
            self.print_time(msg, time)

            msg = " total time spent checking constraints "
            time= self.control.total_time_checking_constraints
            self.print_time(msg, time)

            msg = " total time spent adding reason messages for constraints "
            time= self.control.total_time_adding_constraints_message
            self.print_time(msg, time)

            msg = " total time spent running all loops "
            time= self.control.total_time_running_each_loop
            self.print_time(msg, time)



    def arun_state_feedback_controller(self, time_and_print =True):
        """
        Turned put to be slower due to more method calls or setting a method dynamically
        
        Run the state feedback control strategy the GUI inputs configure the details
        """
        import time
        print("running controller function")
        start_time = time.time()
        data = self.data
        
        start_state = (data.x1_start_controller, data.x2_start_controller)
        max_step = data.max_step_size_controller
        ROS_lower_bounds = self.lower_bound_ROS
        ROS_upper_bounds = self.upper_bound_ROS        
        
        ROS_lower_bounds.reverse()
        ROS_upper_bounds.reverse()
        
        if self.data.upper_guide == "N/A" and self.data.lower_guide == "N/A":
            control_strategy_description = ["linear interpolation control",\
                                            ROS_lower_bounds,\
                                            ROS_upper_bounds]
        else:
            control_strategy_description = ["custom guides",\
                                            self.data.lower_guide,\
                                            self.data.upper_guide]
        """
        control_trajectory,\
        L_values, u_values,\
        u_list_with_state,\
        L_list_with_state,\
        A_list, D_list,\
        A_list_with_state,\
        D_list_with_state \
            = self.control.convex_controller_trajectory(start_state, control_strategy_description, \
                                     epsilon = max_step, endline=1, control_strategy=1)
        """
        print("running this control")

        self.control.select_lambda_x(control_strategy_description)
        control_trajectory, u_list, L_list,\
        A_list, D_list, A_list_with_state,\
        D_list_with_state, u_list_with_state,\
        L_list_with_state, reason \
            = self.control.fast_simulate_trajectory_with_control(start_state, direction=1, T=0.01)

        print("controller run complete")
                
        self.control_trajectory = control_trajectory[0]
        
        end_time =  time.time()    
        seconds = end_time - start_time
        self.fb_control_runtime = seconds

        if time_and_print == True:
            msg = "The runtime for the integration is "
            time = self.control.integration_time
            self.print_time(msg, time)


            msg = " total_time_spent_choosing_actuator limits A and D "
            time= self.control.total_time_choosing_A_D
            self.print_time(msg, time)
            
            msg = " total_time_spent_choosing_actuation_levels "
            time= self.control.total_time_taken_getting_L_values
            self.print_time(msg, time)


            msg = " total time spent appending stuff "
            time= self.control.total_time_appending_values
            self.print_time(msg, time)

            msg = " total time spent calculating move "
            time= self.control.total_time_calculating_move
            self.print_time(msg, time)

            msg = " total time spent checking constraints "
            time= self.control.total_time_checking_constraints
            self.print_time(msg, time)

            msg = " total time spent adding reason messages for constraints "
            time= self.control.total_time_adding_constraints_message
            self.print_time(msg, time)

            msg = " total time spent running all loops "
            time= self.control.total_time_running_each_loop
            self.print_time(msg, time)

    def test_new_constraint_function(self):
        self.control.build_lamdifyed_upper_lower_bound_for_fast_evaluation()
        self.control.build_ufuncify_upper_lower_bound_for_fast_evaluation()        

    def print_time(self, msg, seconds):

        if seconds >= 60: 
            minutes = seconds/60
            print(msg, " ", minutes, " minutes")
        elif seconds < 60:
            print(msg, " ", seconds, " seconds")

    def plot_control_trajectory(self):




        #this is a hacky solution to closing the bounds manually for paper plots
        enclose_bounds = True
        if enclose_bounds==True:
            x1_val = np.linspace(0.0, 0.0, num=1000)
            x2_val = np.linspace(0.0, 6.3, num=1000)

            plt.plot(x1_val, x2_val, '-', color='green', linewidth=3)

            x1_val = np.linspace(1.0, 1.0, num=1000)
            x2_val = np.linspace(4.0, 8.5, num=1000)
            plt.plot(x1_val, x2_val, '-', color='red')

            if self.data.additional_upper_constraint!= "N/A":
                x1_val = [x[0] for x in self.simulation_object.resultant_upper_constraint_curve_data]
                y1_val = [x[1] for x in self.simulation_object.resultant_upper_constraint_curve_data] 
                
                zipped_lists = zip(x1_val, y1_val)
                sorted_pairs = sorted(zipped_lists)
                tuples = zip(*sorted_pairs)
                x1_val, y1_val = [ list(tupl) for tupl in tuples]
                
                plt.plot(x1_val, y1_val, '-', color='green',label="upper resultant", linewidth=3)  

            if self.data.additional_lower_constraint != "N/A":                
                x1_val = [x[0] for x in self.simulation_object.resultant_lower_constraint_curve_data]
                y1_val = [x[1] for x in self.simulation_object.resultant_lower_constraint_curve_data]
                
                zipped_lists = zip(x1_val, y1_val)
                sorted_pairs = sorted(zipped_lists)
                tuples = zip(*sorted_pairs)
                x1_val, y1_val = [ list(tupl) for tupl in tuples]   
            
                plt.plot(x1_val, y1_val, '-', color='green',label="lower resultant", linewidth=3)  



        try:
            x1_val = [x[0] for x in self.control_trajectory]
            y1_val = [x[1] for x in self.control_trajectory]   
            plt.plot(x1_val, y1_val, '-', color='purple',label="control trajectory", ms=3)
        except:
            print("Can't plot control_trajectory")

        try:
            x1_sym = symbols('x1')
            x1_val = np.linspace(0.0, 1.0, num=1000)
            
            x2_val_list = [0]
            for x1 in x1_val:
                x2 = self.data.lower_guide.subs(x1_sym, x1)
                x2_val_list.append(x2)
            x2_val_list.pop(0)
            plt.plot(x1_val, x2_val_list, '-', color='blue',label="lower control guide", ms=2)
        except:
            print("No additional upper control guide available")

        
        try:
            x1_val = [x[0] for x in self.upper_bound_ROS]
            y1_val = [x[1] for x in self.upper_bound_ROS]   
            
            zipped_lists = zip(x1_val, y1_val)
            sorted_pairs = sorted(zipped_lists)
            tuples = zip(*sorted_pairs)
            x1_val, y1_val = [ list(tupl) for tupl in tuples]
            #x1_val, y1_val = self.bubble_sort_lists_for_plotting(x1_val, y1_val)

            plt.plot(x1_val, y1_val, '-', color='orange',label="upper constraint", ms=4)
        except:
            print("Can't plot upper bound")
        
        try:
            x1_val = [x[0] for x in self.lower_bound_ROS]
            y1_val = [x[1] for x in self.lower_bound_ROS]

            zipped_lists = zip(x1_val, y1_val)
            sorted_pairs = sorted(zipped_lists)
            tuples = zip(*sorted_pairs)
            x1_val, y1_val = [ list(tupl) for tupl in tuples]

            #x1_val, y1_val = self.bubble_sort_lists_for_plotting(x1_val, y1_val)            
            plt.plot(x1_val, y1_val, '-', color='orange',label="lower constraint", ms=4)        
        except:
            print("Can't plot lower bound")

        
        try:
            x1_sym = symbols('x1')
            x1_val = np.linspace(0.0, 1.0, num=1000)
            
            x2_val_list = [0]
            for x1 in x1_val:
                x2 = self.data.upper_guide.subs(x1_sym, x1)
                x2_val_list.append(x2)
            x2_val_list.pop(0)
            plt.plot(x1_val, x2_val_list, '-', color='blue',label="upper control guide", ms=2)
        except:
            print("No additional upper control guide available")


        #print(self.control_trajectory)        
        plt.xlabel("$x_1$")
        plt.ylabel("$x_2$")
        #plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.title("Controller plot")
        plt.grid()
        plt.show()
        
class controller_simulation_performance_tests:
    def __init__(self, controller_sim, data):
        self.controller_object = controller_sim
        self.data = data
        
        self.average_fb_controller_times(self.data.number_of_runs_to_average)
        
    
    def average_fb_controller_times(self, times_to_run):
        print("we want to run this controller", times_to_run, "and find the mean")
        gc.disable()
        #create a list of identical objects that we wish to simulate
        controller_objs = [self.controller_object for i in range(times_to_run)]
        
        time_list = [0]
        #loop through each object running a sumlation and storing the times
        j = 0
        while j < times_to_run:
            controller_objs[j].run_state_feedback_controller()
            time_taken = controller_objs[j].fb_control_runtime
            
            time_list.append(time_taken)
            
            j = j + 1
        
        time_list.pop(0)
        
        average = sum(time_list) / len(time_list)
        gc.enable()
        print("Simulations complete, the average time is ", average, " seconds")
        