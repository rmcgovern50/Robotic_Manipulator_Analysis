# -*- coding: utf-8 -*-
"""
Created on Thu Jan 15 2021

@author: Ryan McGovern

This script will use my model of a robot to calculate the dynamics
on a path and plot the admissible and inadmissable regions
"""

import sys
sys.path.append('../../../My_modules/my_basic_modules')
sys.path.append('../../../My_modules/my_control_modules')
sys.path.append('../../../Robotic_manipulator_control')
import functions.ROS_simulations_plotting as plotting
import functions.ROS_algorithm_simulations as sim
import datetime as dt
   
def find_ROS(situation):
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
    
    

    if situation==1:
        """
        situation 1 is the case where:
            - niether extreme trajectories actually intersect the x_2 axis
            - The x1 axis is a boundary between 0 and xa_intersect
            - The td extreme trajectory intersects the constraint curve
        
        This is a good example to show the top boundary algorithm working
        """
        
        folder_name = "example_situation_1/"
        target_state = (0.9, 8.5)
        
        robot = {'joint_masses': [0.25, 0.25],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits': [(-10,10), (-10,10)]\
            }
        simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                              'x2_lim':  [0,20, 0.01],\
                              'line_definition': ["straight line", [(0.34,-0.20),(0.34, 0.20)]],\
                              'initial_state': (0,0),\
                              'final_state': (1,11.5)\
                              }
        print("Situation 1 parameters input")
        
    elif situation==2:
        """
        situation 2 is the case where:
            - the upper extreme trajectory intersect the constraint_curve
            - the lower one intersects the x2_axis
            - the x1 axis is negative
            
        This is a good example for a more obsure case
        
        """       
        
        
        folder_name = "example_situation_2/"
        target_state = (1, 1)
        
        #step 1
        robot = {'joint_masses': [0.25, 10],\
                 'link_lengths': [0.2,0.2],\
                 'actuator_limits':  [(-10,10), (-10,10)]\
                }
        simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                                  'x2_lim':  [0,20, 0.01],\
                                  'line_definition': ["straight line", [(0.34,-0.20),(0.34, 0.20)]],\
                                  'initial_state': (0,0),\
                                  'final_state': (1,11.5)\
                                  }        
        print("Situation 2 parameters input")


    elif situation==3:
        """
        situation 2 is the case where:
            -both extreme trajectories intersect the constraint_curve
            -the x1 axis is negative
            
            
        This is a good example for a more obsure case
        
        """
        
        folder_name = "example_situation_3/"
        target_state = (1, 5)
        
        #step 1
        robot = {'joint_masses': [0.25, 10],\
                 'link_lengths': [0.2,0.2],\
                 'actuator_limits':  [(-10,10), (-10,10)]\
                }
        simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                                  'x2_lim':  [0,20, 0.01],\
                                  'line_definition': ["straight line", [(0.34,-0.20),(0.34, 0.20)]],\
                                  'initial_state': (0,0),\
                                  'final_state': (1,11.5)\
                                  }        
        print("Situation 3 parameters input")

    elif situation==4:
        """
        situation 2 is the case where:
            -both extreme trajectories intersect the constraint_curve
            -the x1 axis is negative
            
            
        This is a good example for a more obsure case        
        """
        
        folder_name = "example_situation_4/"
        target_state = (1, 7.45)
        
        #step 1
        robot = {'joint_masses': [0.25, 10],\
                 'link_lengths': [0.2,0.2],\
                 'actuator_limits':  [(-10,10), (-10,10)]\
                }
        simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                                  'x2_lim':  [0,20, 0.01],\
                                  'line_definition': ["straight line", [(0.34,-0.20),(0.34, 0.20)]],\
                                  'initial_state': (0,0),\
                                  'final_state': (1,11.5)\
                                  }        
        print("Situation 4 parameters input")

    elif situation==5:
        """
        situation 1 is the case where:
            - niether extreme trajectories actually intersect the x_2 axis
            - The x1 axis is a boundary between 0 and xa_intersect
            - The td extreme trajectory intersects the constraint curve
        
        This is a good example to show the top boundary algorithm working
        """

        folder_name = "example_situation_5/"
        target_state = (0.9, 8.5)

        robot = {'joint_masses': [0.25, 0.25],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits': [(-10,10), (-10,10)]\
            }
        simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                              'x2_lim':  [0,20, 0.01],\
                              'line_definition': ["circular arc", [(0.1,0.30), 0.05]],\
                              'initial_state': (0,0),\
                              'final_state': (1,11.5)\
                              }
        print("Situation 5 parameters input")

    elif situation==6:
        """
        situation 6 is the case where:

        This is a good example to show the top boundary algorithm working
        """

        folder_name = "example_situation_6/"
        target_state = (0.9, 8.5)

        robot = {'joint_masses': [0.25, 0.25],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits': [(-10,10), (-10,10)]\
            }
        simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                              'x2_lim':  [0,20, 0.01],\
                              'line_definition': ["circular arc", [(0.1,0.30), 0.05]],\
                              'initial_state': (0,0),\
                              'final_state': (1,11.5)\
                              }
        print("Situation 6 parameters input")




    #step 1
    manipulator = sim.Step_1(current_time,\
                             robot,\
                             simulation_parameters,\
                             run_full_simulation=True,\
                             save_folder=folder_name)
        
    print("Step 1 complete")            
    
    #step 2
    extreme_trajectories,\
    boundaries_from_extreme_trajectories,\
    xa_intersect,\
    xd_intersect,\
    constraint_curve,\
    run_step_3,\
    run_step_4 = \
            sim.Step_2(current_time,\
                       manipulator,\
                       target_state,\
                       run_curve_simulations=True,\
                       run_ROS_boundary_xd_sim=True,\
                       run_ROS_boundary_xa_sim=True,\
                       run_constraint_simulations=True,\
                       run_extract_boundaries_from_extreme_trajectories=True,\
                       save_folder=folder_name,\
                       work_out_steps_to_run = True)
                
    print("Step 2 complete")
    print("intersecting states", xa_intersect, xd_intersect)
    print("Step 3 should be run? ", run_step_3)
    print("Step 4 should be run? ", run_step_4)

    #step 3
    #running is subject to the results from step 2
    if run_step_3:
        lower_boundary_trajectories = sim.Step_3(manipulator,\
                                                 xa_intersect,\
                                                 run_full_simulation=True,\
                                                 save_folder=folder_name)   
        print("Step 3 complete")
    
    #step 4
    #running is subject to step 2 output
    if run_step_4:
        upper_bound_trajectories = sim.Step_4(manipulator, \
                                              xd_intersect,\
                                              xa_intersect, \
                                              constraint_curve,\
                                              run_boundary_sim_setup=True,\
                                              run_boundary_sim=True,\
                                              save_folder=folder_name)
    print("step 4 complete")

    #final plotting functions
    trajectory_list = extreme_trajectories
    trajectory_list.append(constraint_curve)
    trajectory_list.extend(boundaries_from_extreme_trajectories)
    if run_step_3: trajectory_list.append(lower_boundary_trajectories)
    if run_step_4: trajectory_list.extend(upper_bound_trajectories)
    plotting.plot_multiple_trajectories(trajectory_list, current_time, save=False)
    manipulator.plot_workspace_movement(save=False)
    
if __name__ == "__main__":
    
    situation = 6
    find_ROS(situation)
    
    print("all simulations complete")
    
