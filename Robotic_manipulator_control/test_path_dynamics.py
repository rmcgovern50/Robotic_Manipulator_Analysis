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
from robot_models import two_dof_planar_robot
import save_data
import math as m



def test_plot_state_space():

    #Specify robot parameters

    #instatiate class with the physical parameters
    joint_masses = [0.25, 0.25]
    link_lengths = [0.2,0.2]
    actuator_limits = [(-1,1), (-1,1)]
    manipulator = two_dof_planar_robot(joint_masses, link_lengths, actuator_limits )
    

    #define grid to analyse
    s_lim = [0, 1, 0.1]
    sd_lim = [0,20, 0.1]
    #straight_line_definition = [starting_joints, ending_joints]     
    line_definition = [(0.25, 0.15), (-0.25,0.1)]
    manipulator.run_full_path_dynamics_analysis(line_definition, s_lim, sd_lim)
    
    #generate a picture of the admissable region
    manipulator.generate_state_space_plot(manipulator.admissable_region, False)
    
    #simulate the kinematic movements of the robot
    manipulator.simulate_trajectory(manipulator.qs,0.1)
    #generate plots of the movement
    
    manipulator.plot_simulation_parameters([0, "q1 v s"],[0, "q2 v s"], \
                                          [0, "q1, vs q2"], [1, "workspace motion"],\
                                           [0,  "end effector motion"],[0, 50, "robot_gif_motion"]\
                                           , [0, "sub plots q1vq2, workspace"],False)
    
    #generate time optimal control to get from start to end
    #s_plane_coordinates
    initial_coordinate =  (0.55,12)
    final_coordinate = (1,15)
    
    manipulator.generate_time_optimal_trajectory(initial_coordinate, final_coordinate, True) #this part definitely needs work
    
    
    i = 0
    save_all_data(manipulator, i)
    


def save_all_data(manipulator, i):
    save_data.export_tuple_list(manipulator.admissable_region, ("s", "s dot"), "admissible region "+ str(i))
    save_data.export_tuple_list(manipulator.boundry_points, ("s", "s dot"), "admissible region boundry "+ str(i))
    save_data.export_tuple_list(manipulator.s_axisq1, ("s", "q1"), "q1 vs s "+ str(i))
    save_data.export_tuple_list(manipulator.s_axisq2, ("s", "q2"), "q1 vs s "+ str(i))
     
    save_data.export_tuple_list(manipulator.s_plane_control_trajectory, ("s", "s dot"), "control trajectory "+ str(i))

        
if __name__ == "__main__": 

    test_plot_state_space()
    