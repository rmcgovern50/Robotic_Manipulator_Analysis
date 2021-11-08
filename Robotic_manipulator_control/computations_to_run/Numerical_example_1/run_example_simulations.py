# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 14:20:44 2020

@author: Ryan McGovern

This script will use my model of a robot to calculate the dynamics
on a path and plot the admissible and inadmissable regions
"""

import sys
sys.path.append('../../../My_modules/my_basic_modules')   #just incase I want to import some modules
sys.path.append('../../../My_modules/my_control_modules') #just incase I want to import some modules
sys.path.append('../../../Robotic_manipulator_control') #just incase I want to import some modules

import functions.example_simulations_plotting as plotting
import functions.example_simulations as sim

import saved_data.save_data as save_data
import math as m
import datetime as dt
import matplotlib.pyplot as plt


def ex_1_straight_line_joint_space(robot, simulation_parameters, run_full_simulation=False, run_bisection=False):
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots
    
    if run_full_simulation:
        
        manipulator = sim.generate_point_to_point_joint_space(robot,\
                                                  simulation_parameters['x1_lim'],\
                                                  simulation_parameters['x2_lim'],\
                                                  simulation_parameters['line_definition'],\
                                                  simulation_parameters['initial_state'],\
                                                  simulation_parameters['final_state'],\
                                                  current_time)
        save_data.save_obj(manipulator, "numerical_examples/", "manipulator_ex_1")
        
        #get the most extreme trajectories to the target
        extreme_trajectories = sim.generate_extreme_trajectories_to_target(manipulator,simulation_parameters['final_state'])
        save_data.save_obj(extreme_trajectories, "numerical_examples/", "extreme_trajectories_ex_1")
        
    else:
        manipulator = save_data.load_obj("numerical_examples/", "manipulator_ex_1")        
        extreme_trajectories= save_data.load_obj( "numerical_examples/", "extreme_trajectories_ex_1") 
    
    if run_bisection:
        #X0 = (0,7.5)
        #bisection_trajectories = sim.ex_1_find_magic_L(manipulator, X0)
        X0 = (0,7.5)
        bisection_trajectories = sim.auto_find_magic_L(manipulator, X0, extreme_trajectories)
        save_data.save_obj(bisection_trajectories, "numerical_examples/", "bisection_trajectories_ex_1")
    else:
        bisection_trajectories = save_data.load_obj("numerical_examples/", "bisection_trajectories_ex_1") 
    
    #bisection_trajectories = sim.test_intersection_finder(manipulator, X0, extreme_trajectories) 
    plotting.ex_1_produce_plots(manipulator, extreme_trajectories, bisection_trajectories, current_time, save=False)

def run_ex_1():
    robot = {'joint_masses': [0.25, 0.25],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits':  [(-10,10), (-10,10)]\
            }
    
    simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                              'x2_lim':  [0,20, 0.1],\
                              'line_definition': ["joint space line", [(m.pi/2,0),(3*m.pi/2, m.pi/4)]],\
                              'initial_state': (0,0),\
                              'final_state': (1,11.5)\
                              }
    
    ex_1_straight_line_joint_space(robot, simulation_parameters, True, False)
    print("ex 1 simulation_complete")
    
#===============example 2 =============================
def ex_2_straight_line_cartesian_space(robot, simulation_parameters, run_full_simulation=False, run_bisection=False):
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots
    
    if run_full_simulation:
        
        manipulator = sim.straight_line_motion_cartesian_space(robot,\
                                                  simulation_parameters['x1_lim'],\
                                                  simulation_parameters['x2_lim'],\
                                                  simulation_parameters['line_definition'],\
                                                  simulation_parameters['initial_state'],\
                                                  simulation_parameters['final_state'],\
                                                  current_time)
        
        save_data.save_obj(manipulator, "numerical_examples/", "manipulator_ex_2")
        
        #get the most extreme trajectories to the target
        extreme_trajectories = sim.ex_2_generate_extreme_trajectories_to_target(manipulator,simulation_parameters['final_state'])
        save_data.save_obj(extreme_trajectories, "numerical_examples/", "extreme_trajectories_ex_2")
    
    else:
        manipulator = save_data.load_obj("numerical_examples/", "manipulator_ex_2")        
        extreme_trajectories= save_data.load_obj( "numerical_examples/", "extreme_trajectories_ex_2") 
    
    if run_bisection:
        X0 = (0.75, 10.5)
        #bisection_trajectories = sim.ex_2_find_magic_L(manipulator, X0)
        bisection_trajectories = sim.auto_find_magic_L(manipulator, X0, extreme_trajectories)        
        save_data.save_obj(bisection_trajectories, "numerical_examples/", "bisection_trajectories_ex_2")
    else:
        bisection_trajectories = save_data.load_obj("numerical_examples/", "bisection_trajectories_ex_2") 
    
    plotting.ex_2_produce_plots(manipulator, extreme_trajectories, bisection_trajectories, current_time)
    print("done") 

def run_ex_2():
    robot = {'joint_masses': [0.25, 0.25],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits':  [(-10,10), (-10,10)]\
            }
    simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                              'x2_lim':  [0,30, 0.1],\
                              'line_definition': ["straight line" ,[(0.35,0),(0, 0.25)]],\
                              'initial_state': (0,0),\
                              'final_state': (1,11.5)\
                              }
    ex_2_straight_line_cartesian_space(robot, simulation_parameters, True, True)
    print("ex 2 simulation_complete")

    
def approximate_region_of_attraction_ex_2():
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots    
    
    manipulator = save_data.load_obj("numerical_examples/", "manipulator_ex_2")
    extreme_trajectories= save_data.load_obj( "numerical_examples/", "extreme_trajectories_ex_2")
    
    Tl, Tu = sim.ex_2_approx_roa(manipulator, extreme_trajectories)
    print(Tl, Tu)
    plotting.ex_2_produce_plots_roa(manipulator, Tl, Tu, extreme_trajectories, current_time, False)

#=================================== ex 3 =====================================

def run_ex_3():
    """
    This is a function for working out how heavy the end effector should be to slow the robot down
    while the input is asking for maximum acceleration
    """
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots    

    simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                              'x2_lim':  [0,30, 0.1],\
                              'line_definition': ["straight line", [(0.34,-0.20),(0.34, 0.20)]],\
                              'initial_state': (0,0),\
                              'final_state': (1,11.5)\
                              }

    increasing_end_effector_mass_sim(simulation_parameters, current_time,\
                                     create_manipulator_selection=True)

def ex_3_loop_end_effector_mass(simulation_parameters, current_time, create_manipulator_selection=False):

    #define the robot description that will be varied
    robot = {'joint_masses': [0.25, 0.25],\
     'link_lengths': [0.2,0.2],\
     'actuator_limits':  [(-10,10), (-10,10)]\
    }
    
    #define these end effector masses
    start_mass = robot['joint_masses'][1]
    end_mass = 1000*robot['joint_masses'][1]

    number_of_increments = 2
    #define a list to pass
    sim_range_list = [start_mass, end_mass, number_of_increments]

    if create_manipulator_selection:
        manipulators = sim.make_selection_with_varying_end_effector_mass(robot, sim_range_list, simulation_parameters, current_time)
        save_data.save_obj(manipulators, "numerical_examples/", "manipulator_data_ex_3")
    else:
        manipulators = save_data.load_obj("numerical_examples/", "manipulator_data_ex_3")

    return  manipulators, sim_range_list 
    
def increasing_end_effector_mass_sim(simulation_parameters, current_time, create_manipulator_selection=False):

    manipulators, sim_range_list = ex_3_loop_end_effector_mass(simulation_parameters, current_time,  create_manipulator_selection)
    T_list = sim.shoot_trajectory_for_each_manipulator(manipulators) 
    #print(len(T_list))
    plotting.ex_3_produce_plots(manipulators, simulation_parameters, T_list, sim_range_list, current_time)

if __name__ == "__main__": 
    #run_ex_1()
    #run_ex_2()
    #approximate_region_of_attraction_ex_2()
    run_ex_3()
    
    
    print("all simulations complete")
    
    