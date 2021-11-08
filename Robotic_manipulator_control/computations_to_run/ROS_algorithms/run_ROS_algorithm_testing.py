# -*- coding: utf-8 -*-
"""
Created on Thu Jan 15 2021

@author: Ryan McGovern

This script will use my model of a robot to calculate the dynamics
on a path and plot the admissible and inadmissable regions
"""

import sys
sys.path.append('../../../My_modules/my_basic_modules')   #just incase I want to import some modules
sys.path.append('../../../My_modules/my_control_modules') #just incase I want to import some modules
sys.path.append('../../../Robotic_manipulator_control') #just incase I want to import some modules

#import os # Import the os module
#path_parent = os.path.dirname(os.getcwd())
#path_grandparent = os.path.dirname(path_parent)
#os.chdir(path_grandparent)#change the directory this is run from so that all the relevant folders are easily accessed 

#sys.path.append(path_grandparent) #just incase I want to import some modules

import functions.ROS_simulations_plotting as plotting
import functions.ROS_algorithm_testing_simulations as sim

import save_data as save_data
import math as m
import datetime as dt
import matplotlib.pyplot as plt


def run_find_positive_x1_axis():
    robot = {'joint_masses': [0.25, 0.25],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits': [(-10,10), (-10,10)]\
            }
    simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                              'x2_lim':  [0,20, 0.1],\
                              'line_definition': ["straight line", [(0.34,-0.20),(0.34, 0.20)]],\
                              'initial_state': (0,0),\
                              'final_state': (1,11.5)\
                              }
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots       
    
    manipulator = create_simulation_manipulator(robot, simulation_parameters, current_time, run_full_simulation=True)
    extreme_trajectories = create_trajectories_to_target(manipulator, current_time, run_full_simulation=True)
    evaluate_max_input_between_zero_and_Ta(manipulator, extreme_trajectories, current_time)
    
    print("first simulation_complete")
    
def create_simulation_manipulator(robot, simulation_parameters, current_time, run_full_simulation=False):
    """
    function to create a manipulator and plot
    """

    if run_full_simulation:    
        manipulator = sim.generate_point_to_point(robot,\
                                                  simulation_parameters['x1_lim'],\
                                                  simulation_parameters['x2_lim'],\
                                                  simulation_parameters['line_definition'],\
                                                  simulation_parameters['initial_state'],\
                                                  simulation_parameters['final_state'],\
                                                  current_time)    
        save_data.save_obj(manipulator, "ROS data/", "manipulator")        
    else:
        manipulator = save_data.load_obj("ROS data/", "manipulator")        
    
    return manipulator
    
    
def create_trajectories_to_target(manipulator, current_time, run_full_simulation=False):
    """
    function to create extrene trajectories to the goal and plot
    """
    if run_full_simulation:
        target_state = (0.75, 10.5)
        extreme_trajectories = sim.generate_extreme_trajectories_to_target(manipulator, target_state)
        save_data.save_obj(extreme_trajectories, "ROS data/", "extreme_trajectories")    
    else:
        extreme_trajectories = save_data.load_obj("ROS data/", "extreme_trajectories")        
    plotting.produce_extreme_trajectory_plots(extreme_trajectories, current_time)
    
    return extreme_trajectories
    
def evaluate_max_input_between_zero_and_Ta(manipulator, extreme_trajectories, current_time):
    """
    function to evaluate u(x, L=1) along the x1 axis between zero an Ta
    """    

    Ta_axis_intersect_x1_value = extreme_trajectories[1][-1][0]
    print("axis intersect ", Ta_axis_intersect_x1_value)
    evaluated_list = sim.perform_u_scan(manipulator, Ta_axis_intersect_x1_value)
    #print(evaluated_list)


def simulate_a_trajectory():
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots           
    manipulator = save_data.load_obj("ROS data/", "manipulator")            
    extreme_trajectories = save_data.load_obj("ROS data/", "extreme_trajectories")    
    
    start_state = (0.2,4)
    L=1
    T = sim.generate_a_trajectory(manipulator, start_state, L)
    print(T)
    plotting.produce_extreme_trajectory_plots_with_extra(T, extreme_trajectories, current_time)
        


def find_ROS():
    """
    This function will systematically run all simulations required to find the
    ROS
    #Step 1 - create a manipulator object
    #Step 2 - form the extreme trajectories to the target
    #Step 3 - Form any extensions on the lower bound targets
    #Step 4 - Form any extensions on the upper bound targets
    
    The objective will be to perform the steps and plot the result after each
    """
            
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots

    #step 1
    manipulator = sim.Step_1(current_time,\
                             run_full_simulation=False) 
    #step 2
    extreme_trajectories,\
    boundaries_from_extreme_trajectories,\
    xa_intersect,\
    xd_intersect,\
    constraint_curve = \
            sim.Step_2(current_time,\
                       manipulator,\
                       run_curve_simulations=False,\
                       run_ROS_boundary_xd_sim=False,\
                       run_ROS_boundary_xa_sim=False,\
                       run_constraint_simulations=False,\
                       run_extract_boundaries_from_extreme_trajectories=False)   
    #step 3
    lower_boundary_trajectories = sim.Step_3(manipulator,\
                                             xa_intersect,\
                                             run_full_simulation=False)   
    #step 4
    upper_bound_trajectories = sim.Step_4(manipulator, \
                                          xd_intersect, \
                                          constraint_curve,\
                                          run_boundary_sim_setup=False,\
                                          run_boundary_sim=False)
    
    #final plotting functions
    trajectory_list = extreme_trajectories
    trajectory_list.append(constraint_curve)
    trajectory_list.extend(boundaries_from_extreme_trajectories)
    trajectory_list.append(lower_boundary_trajectories)
    trajectory_list.extend(upper_bound_trajectories)
    
    plotting.plot_multiple_trajectories(trajectory_list, current_time, save=False)
    
    
if __name__ == "__main__": 
    
    find_ROS()
    #run_find_positive_x1_axis()        
    #simulate_a_trajectory()

    print("all simulations complete")
    
    
