# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 14:20:44 2020

@author: Ryan McGovern

This script will use my model of a robot to calculate the dynamics
on a path and plot the admissible and inadmissable regions
"""

import sys
sys.path.append('../My_modules/my_basic_modules')   #just incase I want to import some modules
sys.path.append('../My_modules/my_control_modules') #just incase I want to import some modules

import os # Import the os module
path_parent = os.path.dirname(os.getcwd())
path_grandparent = os.path.dirname(path_parent)
os.chdir(path_grandparent)#change the directory this is run from so that all the relevant folders are easily accessed 


#import functions.example_simulations_plotting as plotting
import functions.lipschitz_simulations as lipschitz_sim


import saved_data.save_data as save_data
import math as m
import datetime as dt
#import matplotlib.pyplot as plt
from robot_models import two_dof_planar_robot

def check_lipschitz_condition(run_full_simulation=True):
    
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots   
    if run_full_simulation:
        robot_parameters = {'joint_masses': [0.25, 0.25],\
                 'link_lengths': [0.2,0.2],\
                 'actuator_limits':  [(-10,10), (-10,10)]\
                }
        
        simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                                  'x2_lim':  [0,20, 0.1],\
                                  'path_definition': ["joint space line", ((m.pi/2,0),(3*m.pi/2, m.pi/4))],\
                                  'initial_state': (0,0),\
                                  'final_state': (1,11.5)\
                                  }    
        x1_lim = simulation_parameters['x1_lim']
        x2_lim = simulation_parameters['x2_lim']
        path_def = simulation_parameters['path_definition']
        manipulator = two_dof_planar_robot(robot_parameters, current_time)
        manipulator.run_full_path_dynamics_analysis(path_def, x1_lim, x2_lim)
        
        save_data.save_obj(manipulator, "lipschitz_data/", "manipulator")
        
    else:
        manipulator = save_data.load_obj("lipschitz_data/", "manipulator")

    lipschitz_sim.run_lipschitz_region_calculations(manipulator)

    
    
    
if __name__ == "__main__": 
    check_lipschitz_condition(run_full_simulation=False)
    
    print("all simulations complete")
    
    