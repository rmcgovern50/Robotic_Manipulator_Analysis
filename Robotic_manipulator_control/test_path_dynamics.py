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
import math as m



def test_plot_state_space():

    #Specify robot parameters

    #instatiate class with the physical parameters
    manipulator = two_dof_planar_robot([0.1, 0.1], [0.2,0.2], [(-1,1), (-1,1)])
    
   
    #straight_line_definition = [starting_joints, ending_joints]     
    line_definition = [(-0.2,-0.15), (0.3,-0.25)]
    

    #define grid to analyse
    s_lim = [0, 1, 0.1]
    sd_lim = [0,10, 0.1]
    manipulator.run_full_path_dynamics_analysis(line_definition, s_lim, sd_lim)
    
    #generate a picture of the admissable region
    manipulator.generate_state_space_plot(manipulator.admissable_region, False)
    
    #simulate the kinematic movements of the robot
    manipulator.simulate_trajectory(manipulator.qs,0.1)
    #generate plots of the movement
    
    manipulator.plot_simulation_parameters(1,1,1,1,1,[1, 50, "linear motion"], 0,True)
    
    
    #manipulator.generate_bound_vector_plot([(0,2), (0.1,5), (0.2,2),(0.6,6),(0.8,0.4),])
    
    
    #generate time optimal control to get from start to end
    trajectory = manipulator.generate_time_optimal_trajectory(True) #this part definitely needs work

   
        
if __name__ == "__main__": 

    test_plot_state_space()
    