# -*- coding: utf-8 -*-
"""
@author: Ryan
This file contains methods that produce plots
"""

from robot_data_visualisation import two_dof_robot_data_visualisation
import numpy as np
import time

def ex_1_produce_plots(manipulator, trajectory_list, bisection_trajectories, current_time, save=True):
    
    admissible_region = manipulator.admissible_region
    admissible_region_grid = [manipulator.s_lims, manipulator.sd_lims]
    s_axisq1 = manipulator.s_axisq1
    s_axisq2 = manipulator.s_axisq2
    coordinates_q1_q2 = manipulator.coordinates_q1_q2
    link_lengths = manipulator.link_lengths
    x_y_coordinates = manipulator.x_y_coordinates
                  
    plotter = two_dof_robot_data_visualisation(current_time)
    
    plotter.generate_state_space_plot(admissible_region, True, 1, "admissible_region", "numerical_example_1/")
    
    plotter.plot_q1_against_q2(coordinates_q1_q2,True,5,"joint_space", "numerical_example_1/")
    plotter.plot_robot_motion_x_y(link_lengths, x_y_coordinates, s_axisq1, True,5,"workspace","numerical_example_1/")
    
    
    label_list = ['T_l', 'T_a']
    color_list = ['g', 'b']
    
    #print("hello")
    plotter.overlay_trajectories_with_admissible_region("N/A",\
                                                        trajectory_list,\
                                                        label_list,color_list,\
                                                        save,1,"numerical_example_1/",\
                                                        "extreme_trajectory_plot")

    label_list = ['T_l', 'T_a', 'T1', 'T2', 'T3', 'T4', 'T5', 'T6', 'T7' , 'T8', 'T9', 'T10']
    color_list = ['c', 'c', 'y', 'y', 'y', 'y', 'y', 'y','y','y','y', 'g']
    trajectory_list.extend(bisection_trajectories)

    plotter.overlay_trajectories_with_admissible_region("N/A",\
                                                        trajectory_list,\
                                                        label_list,color_list,\
                                                        save,1,"numerical_example_1/",\
                                                        "bisection_plot")    
    
#============example 2=================================================================
    
def ex_2_produce_plots(manipulator, trajectory_list, bisection_trajectories, current_time, save=True):
    
    admissible_region = manipulator.admissible_region
    admissible_region_grid = [manipulator.s_lims, manipulator.sd_lims]
    s_axisq1 = manipulator.s_axisq1
    s_axisq2 = manipulator.s_axisq2
    coordinates_q1_q2 = manipulator.coordinates_q1_q2
    link_lengths = manipulator.link_lengths
    x_y_coordinates = manipulator.x_y_coordinates
                  
    plotter = two_dof_robot_data_visualisation(current_time)
    
    plotter.generate_state_space_plot(admissible_region, True, 1, "admissible_region", "numerical_example_2/")
    
    plotter.plot_q1_against_q2(coordinates_q1_q2,True,5,"joint_space", "numerical_example_2/")
    plotter.plot_robot_motion_x_y(link_lengths, x_y_coordinates, s_axisq1, True,5,"workspace","numerical_example_2/")  
    
    
    label_list = ['T_l', 'T_a']
    color_list = ['g', 'b']
    plotter.overlay_trajectories_with_admissible_region(manipulator.admissible_region,\
                                                        trajectory_list,\
                                                        label_list,color_list,\
                                                        save,1,"numerical_example_2/",\
                                                        "extreme_trajectory_plot")    
    
    label_list = ['T_l', 'T_a', 'T1', 'T2', 'T3', 'T4', 'T5', 'T6', 'T7' , 'T8', 'T9', 'T10']
    color_list = ['c', 'c', 'y', 'y', 'y', 'y', 'y', 'y','y','y','y', 'g']
    trajectory_list.extend(bisection_trajectories)
    plotter.overlay_trajectories_with_admissible_region(manipulator.admissible_region,\
                                                        trajectory_list,\
                                                        label_list,color_list,\
                                                        save,10,"numerical_example_2/",\
                                                        "bisection_plot")



def ex_2_produce_plots_roa(manipulator, trajectory_list_lower, trajectory_list_upper, extreme_trajectories, current_time, save=True):
    
    plotter = two_dof_robot_data_visualisation(current_time)
    label_list = ['T_l', 'T_a', 'T1', 'T2', 'T3', 'T4', 'T5', 'T6', 'T7' , 'T8', 'T9', 'T10']
    color_list = ['c', 'c', 'y', 'y', 'y', 'y', 'y', 'y','y','y','y', 'g']
    extreme_trajectories.append(trajectory_list_lower)
    extreme_trajectories.extend(trajectory_list_upper)
    #trajectory_list_lower.extend(extreme_trajectories)
    #trajectory_list_lower.extend(trajectory_list_upper)
    plotter.overlay_trajectories_with_admissible_region(manipulator.admissible_region,\
                                                        extreme_trajectories,\
                                                        label_list,color_list,\
                                                        save,10,"numerical_example_2/",\
                                                        "roa_plot")


        
def ex_2b_lower_bisection_produce_plots(manipulator, trajectories_to_final_state, new_extremes, current_time):
                  
    plotter = two_dof_robot_data_visualisation(current_time)
    
    label_list = ['T_l', 'T_a', 'T1', 'T2', 'T3', 'T4', 'T5', 'T6', 'T7']
    color_list = ['c', 'c', 'y', 'y', 'y', 'y', 'y','m', 'g']
    trajectories_to_final_state.extend(new_extremes)
    plotter.overlay_trajectories_with_admissible_region(manipulator.admissible_region,\
                                                        trajectories_to_final_state,\
                                                        label_list,color_list,\
                                                        False,1,"numerical_example_2/",\
                                                        "region_of_attraction")
    
    
    

def ex_3_produce_plots(manipulators, simulation_parameters, trajectories, sim_range_list, current_time):
 
    plotter = two_dof_robot_data_visualisation(current_time)
    start = sim_range_list[0]
    stop = sim_range_list[1]
    number_of_points = sim_range_list[2]
    line_definition = simulation_parameters['line_definition']
    
    sim_points = np.linspace(start, stop, number_of_points)
    print(len(manipulators),len(trajectories),len(sim_points))    
    
    for manipulator, trajectory, mass  in zip(manipulators, trajectories, sim_points):
        
        admissible_region = manipulator.admissible_region
        admissible_region_grid = [manipulator.s_lims, manipulator.sd_lims]
        s_axisq1 = manipulator.s_axisq1
        s_axisq2 = manipulator.s_axisq2
        coordinates_q1_q2 = manipulator.coordinates_q1_q2
        link_lengths = manipulator.link_lengths
        x_y_coordinates = manipulator.x_y_coordinates
                      
        #plot the admissible region
        plotter.generate_state_space_plot(admissible_region, save=False, marker_size=1, title="state space at " + str(mass) + "kg")
        plotter.plot_robot_motion_x_y(link_lengths, x_y_coordinates, s_axisq1, False)    
        print("mass = ", mass)
        
        label_list = ['T_l', 'T_a', 'T1', 'T2', 'T3', 'T4', 'T5', 'T6', 'T7']
        color_list = ['c', 'c', 'y', 'y', 'y', 'y', 'y','m', 'g']
        
        title_string = "max acceleration trajectory at " + str(mass) + "kg \n cartesian space line definition = " + str(line_definition[0]) + "->"  + str(line_definition[0]) 
        plotter.overlay_trajectories_with_admissible_region(manipulator.admissible_region,\
                                                            [trajectory],\
                                                            label_list,color_list,\
                                                            save=True,\
                                                            filepath= "overlayed_plots/",\
                                                            filename= "overlayed_trajectories "+str(mass) + "kg",\
                                                            title=title_string)


