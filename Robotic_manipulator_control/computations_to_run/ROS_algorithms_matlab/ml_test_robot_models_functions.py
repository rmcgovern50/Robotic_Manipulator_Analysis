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

import math as m
from math import pi
import matlab.engine
import numpy as np

import os
cwd = os.getcwd()

eng = matlab.engine.start_matlab()
s = eng.genpath(cwd)
eng.addpath(s, nargout=0)

import functions.ROS_simulations_plotting as plotting
import functions.ml_ROS_algorithm_simulations as sim
import datetime as dt
import numpy as np
from my_sorting import combine_to_tuples
from functions.ml_controller import path_dynamics_controller

import save_data as save_data


def test_function(situation, current_time):
        
    if situation == 1:
        folder_name = "ml_example_1/"
        
        simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                              'x2_lim':  [0, 20, 0.1],\
                              'robot': "universalUR5",\
                              'path_definition': ["joint_space_straight_line", [[pi/2, pi/2, pi/3, 2*pi/3, -pi/2, -pi/3,]], [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]] ]
                               }
    
    #step 1
    manipulator = sim.Step_1(current_time,
                             simulation_parameters,\
                             eng,\
                             run_full_simulation=False,\
                             save_folder=folder_name)
    
    manipulator.plot_admissible_region()
    
    x1 = 0.0
    x2 = 0.0   
    qx1_evaluated, dqx1_evaluated, ddqx1_evaluated =\
    manipulator.evaluate_path_parameters(x1, x2)
    
    admissible = eng.check_if_admissible("universalUR5", \
                         eng.double(x1), \
                         eng.double(x2), \
                         qx1_evaluated,  \
                         dqx1_evaluated, \
                         ddqx1_evaluated)
        
    limits = eng.calc_A_D("universalUR5", eng.double(x1), \
                          eng.double(x2), \
                          qx1_evaluated, \
                          dqx1_evaluated, \
                          ddqx1_evaluated)
    A = limits[0][0]
    D = limits[0][1]        
    print("1: ", -A + D)
    
    
    
    control = path_dynamics_controller(manipulator)
    state = (x1, x2)
    A, D = control.calc_upper_lower_bound_values(state)
    
    print("2: ", -A + D)
        
    #if D>A:
    #    print("comparison worked")    
    
    #print("whats up", (A, D), "yo" ,admissible)
    
    print("start")
    target_state = (0.8, 2)
    T, reason = sim.form_tajectory(manipulator, target_state)
    print(len(T[-1]), reason)
    print("end")
    plotting.plot_multiple_trajectories([T], current_time, "paper_plots/", save=False)
    
    #wayPoints = manipulator.create_circular_arc_waypoints()
    #print(wayPoints)
    
    #q = manipulator.ik_solver(wayPoints)
    #print("configs: ", q)
    
    #result1 = manipulator.evaluate_expression( manipulator.qx1,  [ (manipulator.x1, 0.1) ] )       
    #print("result 1 = ", result1)
    
    #result2 = manipulator.evaluate_expression( manipulator.qds, [(manipulator.x1, 0.1), (manipulator.x2,  3)])
    #print("result 2 = ", result2)

    #result3 = manipulator.evaluate_expression( manipulator.qdds, [(manipulator.x1, 0.1), (manipulator.x2,  3) , (manipulator.u, 1)])
    #print("result 3 = ", result3)
    
if __name__ == "__main__": 

    start_time =  dt.datetime.now()
    situation = 1
    
    test_function(situation, start_time)
    
    end_time =  dt.datetime.now()    
    diff = end_time-start_time
    minutes = diff.seconds/60
    seconds = diff.seconds
    if seconds >= 60: 
        minutes = diff.seconds/60
        print("simulation took ", minutes, " minutes")
    elif seconds < 60:
        print("simulation took ", diff.seconds, " seconds")

    eng.quit()