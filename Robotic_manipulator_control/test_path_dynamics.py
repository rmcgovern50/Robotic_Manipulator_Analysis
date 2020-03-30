# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 14:20:44 2020

@author: Ryan McGovern

This script will use my model of arobot to calculate the dynamics
on a path and plot the admissable and inadmissable regions
"""

import sys
sys.path.append('../My_modules/my_basic_modules') #just incase I want to import some modules
sys.path.append('../My_modules/my_control_modules') #just incase I want to import some modules
from my_visualising import simple_plot
from robot_models import revolute_prismatic






def test_plot_state_space():

    #Specify robot parameters

    #instatiate class with the physical parameters
    manipulator = revolute_prismatic(0.1, [(-1,1), (-1,1)])
    straight_line_definition = [(0.1, 1), (1, 1)] 
    #define grid to analyse
    s_lim = [0, 1, 0.1]
    sd_lim = [0,10, 0.1]
    
    manipulator.Run_path_dynamics(straight_line_definition, s_lim, sd_lim)
    trajectory = manipulator.generate_time_optimal_trajectory()


    
    
        
if __name__ == "__main__": 
    test_plot_state_space()