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

from robot_models import revolute_prismatic, two_dof_planar_prismatic




def test_plot_state_space2():

    #Specify robot parameters

    #instatiate class with the physical parameters
    manipulator = two_dof_planar_prismatic([0.1, 0.1], [0.2,0.2], [(-1,1), (-1,1)])
    #manipulator.check_if_dynamics_valid()
    
    #efine the starting and ending angles
    straight_line_definition = [(0.7, 0.15), (-0.15, -0.7)] 


    #define grid to analyse
    s_lim = [0, 1, 0.1]
    sd_lim = [0,10, 0.1]
    
    manipulator.run_full_path_dynamics_analysis(straight_line_definition, s_lim, sd_lim)
    
    manipulator.generate_state_space_plot(manipulator.admissable_region)
    manipulator.plot_end_effector_trajectory(manipulator.qs,0.1,1,1,1,1)
    
    trajectory = manipulator.generate_time_optimal_trajectory(True)











def test_plot_state_space():

    #Specify robot parameters

    #instatiate class with the physical parameters
    manipulator = revolute_prismatic(0.1, [(-1,1), (-1,1)])
    manipulator.check_if_dynamics_valid()
    
    straight_line_definition = [(0.1, 1), (1, 1)] 
    #define grid to analyse
    s_lim = [0, 1, 0.1]
    sd_lim = [0,10, 0.1]
    
    manipulator.run_full_path_dynamics_analysis(straight_line_definition, s_lim, sd_lim)
    #manipulator.s_sdot_plot.show()
    #trajectory = manipulator.generate_time_optimal_trajectory()


    
    
        
if __name__ == "__main__": 
    #test_plot_state_space()
    test_plot_state_space2()
    