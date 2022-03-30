# -*- coding: utf-8 -*-
"""
Created on Thu Jan 15 2021

@author: Ryan McGovern

This script will use my model of a robot to calculate the dynamics
on a path and plot the admissible and inadmissable regions
"""

import sys
sys.path.append('../../../My_modules/my_basic_modules') #just incase I want to import some modules
sys.path.append('../../../My_modules/my_control_modules') #just incase I want to import some modules
sys.path.append('../../../Robotic_manipulator_control') #just incase I want to import some modules

import math as m
from math import pi
import matlab.engine
import numpy as np

import functions.ml_ROS_algorithm_simulations as sim
import datetime as dt
import numpy as np
from my_sorting import combine_to_tuples
from functions.ml_controller import path_dynamics_controller
import functions.ml_ROS_simulations_plotting as plotting

import save_data as save_data

start_time =  dt.datetime.now()

import os
cwd = os.getcwd()

print("Initialising matlab engine ")
eng = matlab.engine.start_matlab()
s = eng.genpath(cwd)
eng.addpath(s, nargout=0)

end_time =  dt.datetime.now()    
diff = end_time-start_time
seconds = diff.seconds

if seconds >= 60: 
    minutes = diff.seconds/60
    print("matlab engine took ", minutes, " minutes to set up")
elif seconds < 60:
    print("matlab engine took ", diff.seconds, " seconds to set up")

def find_ROS(situation, current_time):

    if situation == 1:
        
        folder_name = "ml_paper_example_1/"
        
        "always go [upper lower]"
        target_interval = [(0.9, 4.0), (0.9, 2.0)]
        
        simulation_parameters = { 'x1_lim': [0, 1, 0.01],\
                                 'x2_lim':  [0, 20, 0.05],\
                                 'robot': "universalUR5",\
                                 'path_definition': ["joint_space_straight_line", [[pi/2, pi/2, pi/3, 2*pi/3, -pi/2, -pi/3,], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]]
                               }
        # 'path_definition': ["joint_space_straight_line", [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], [[pi/2, pi/2, pi/3, 2*pi/3, -pi/2, -pi/3,]] ]
        print("ml example parameters input")

    elif situation == 2:
        
        folder_name = "ml_paper_example_2/"
        
        "always go [upper lower]"
        target_interval = [(0.9, 4.0), (0.9, 2.0)]
        
        simulation_parameters = { 'x1_lim':[0, 0.99, 0.01],\
                                 'x2_lim':  [0,20, 0.05],\
                                 'robot': "universalUR5",\
                                 'path_definition': ["joint_space_straight_line", [[pi/2, pi/2, pi/3, 2*pi/3, -pi/2, -pi/3,], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]]
                               }
        # 'path_definition': ["joint_space_straight_line", [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]], [[pi/2, pi/2, pi/3, 2*pi/3, -pi/2, -pi/3,]] ]
        print("ml example parameters input")
        
    print("situation: ", situation)
    #step 1
    manipulator = sim.Step_1(current_time,
                             simulation_parameters,\
                             eng,\
                             run_full_simulation=True,\
                             save_folder=folder_name)
    
    #manipulator.plot_admissible_region()
    
    print("step 1 complete: ", manipulator)
    
    control = path_dynamics_controller(manipulator)
    new_constraint_list = sim.form_constraints_list(situation)    
    control.set_constraint_list(new_constraint_list)
    
    extreme_trajectories,\
    xa_intersect,\
    xd_intersect,\
    constraint_curve,\
    run_step_3,\
    run_step_4 = sim.Step_2(manipulator, \
                            target_interval,\
                            situation, control,\
                            run_curve_simulations=True, \
                            work_out_steps_to_run=True, \
                            run_constraint_simulations=True,\
                            save_folder=folder_name)
    
    extreme_trajectories.append(constraint_curve)
    
    print("Step 2 complete")
    
    print("Step 3 should be run? ", run_step_3)
    print("Step 4 should be run? ", run_step_4)
    
    print("xa", xa_intersect)
    print("xd", xd_intersect)   
    
    #print("extreme traj: ", len(extreme_trajectories))
    #plotting.plot_multiple_trajectories(target_interval, extreme_trajectories, current_time, "paper_plots/", save=True)

    #step 3
    #running is subject to the results from step 2
    #run_step_3 = True
    #run_step_4 = True
    
    if run_step_3:
        lower_boundary_trajectories = sim.Step_3(manipulator,\
                                                 control,\
                                                 xa_intersect,\
                                                 run_full_simulation=True,\
                                                 save_folder=folder_name)
        
        print("Step 3 complete")
        extreme_trajectories.append(lower_boundary_trajectories)
        #plotting.plot_multiple_trajectories(target_interval, extreme_trajectories, current_time, "paper_plots/", save=False)
    
    #step 4
    #running is subject to step 2 output
    
    if run_step_4:
        upper_bound_trajectories = sim.Step_4(manipulator,\
                                              control,\
                                              xd_intersect,\
                                              xa_intersect, \
                                              constraint_curve,\
                                              situation,\
                                              extreme_trajectories,\
                                              run_boundary_sim_setup=True,\
                                              run_boundary_sim=True,\
                                              save_folder=folder_name)
        
        print("step 4 complete")
        
        loop = 0
        
        #print("number of trajectories: ", len(upper_bound_trajectories))
        
        for up_traj in upper_bound_trajectories:
            extreme_trajectories.append(up_traj)
            loop = loop + 1
        plotting.plot_region_boundaries(target_interval, extreme_trajectories, current_time, folder_name, save=False, heading= "Reach avoid set")
    
    create_controller = True
    
    if create_controller==True:
        """
        Note,
        paper uses 
        control type 1 as boundary of R as guides
        control type 6 guides given by two different order polynomials
        control type 4 as a simple single linear guide      
        """
        
        control_type = "1"
        simulate_controller = True
        
        start_state = (0.1, 2)        
        x1_target = target_interval[0][0]
        
        sym_x1 = control.s 
        new_constraint_list = [-sym_x1 + start_state[0],\
                               sym_x1 - x1_target,\
                               control.constraint_list[2],\
                               control.constraint_list[3],\
                               control.constraint_list[4] \
                                ]
        
        control.set_constraint_list(new_constraint_list)
        
        control_trajectory, u_values = \
            sim.create_control_trajectory(current_time,\
                                      control_type,\
                                      control,\
                                      start_state,\
                                      extreme_trajectories, \
                                      lower_boundary_trajectories, \
                                      upper_bound_trajectories, \
                                      simulate_controller,\
                                      target_interval,\
                                      save_folder=folder_name)

        plotting.plot_region_boundaries_with_control(target_interval, \
                                                     extreme_trajectories, \
                                                     current_time, folder_name,\
                                                     con_traj=control_trajectory[0],\
                                                     save=False,\
                                                     heading= "Reach avoid set")
        
    create_joint_plots = True
    
    if create_joint_plots == True:
        path_def = simulation_parameters['path_definition']
        sim.create_joint_plots(path_def)
        
    #print("the ", len(control_trajectory[0]))
    print(len(u_values))
    switch = True
    torque_list = [[0]]
    q_list = [0]
    if switch == True:
        
        x1_list = [x[0] for x in control_trajectory[0]]
        x2_list = [x[1] for x in control_trajectory[0]]
        
        for x1, x2 in zip(x1_list, x2_list):
            qx1_evaluated,\
            dqx1_evaluated,\
            ddqx1_evaluated \
            = manipulator.evaluate_path_parameters(x1, x2)
            
            #print(x1, x2)
            torques = eng.Calc_torque_required("universalUR5",\
                                         eng.double(float(x1)), \
                                         eng.double(float(x2)), \
                                         qx1_evaluated, \
                                         dqx1_evaluated, \
                                         ddqx1_evaluated, \
                                         float(0))
            
                
                
            torque_list_el = [ torques[0][0] ,torques[1][0], torques[2][0],\
                           torques[3][0], torques[4][0], torques[5][0] ]
            #print("q ", qx1_evaluated)
            q_list.append(qx1_evaluated)
            torque_list.append(torque_list_el)
            
    q_list.pop(0)
    torque_list.pop(0)
    print("joints start ", q_list[0])
    print("joints end ", q_list[-1])
    #[[pi/2, pi/2, pi/3, 2*pi/3, -pi/2, -pi/3]]
    #[[0.0,  0.0,  0.0,  0.0,     0.0,   0.0]]]
    #print("torques " , len(torque_list), len(torque_list[2]))
    plotting.plot_torques(torque_list)
    plotting.plot_joint_list(q_list)
    
    
    
if __name__ == "__main__": 

    start_time =  dt.datetime.now()
    situation = 2
    
    find_ROS(situation, start_time)
    
    end_time =  dt.datetime.now()    
    diff = end_time-start_time
    seconds = diff.seconds
    
    if seconds >= 60: 
        minutes = diff.seconds/60
        print("simulation took ", minutes, " minutes")
    elif seconds < 60:
        print("simulation took ", diff.seconds, " seconds")
    
    print("all simulations complete")

    eng.quit()
    print("tests complete")
    
    