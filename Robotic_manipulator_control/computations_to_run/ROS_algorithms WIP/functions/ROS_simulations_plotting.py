# -*- coding: utf-8 -*-
"""
@author: Ryan
This file contains methods that produce plots
"""

from robot_data_visualisation import two_dof_robot_data_visualisation
import numpy as np
import time
import matplotlib.pyplot as plt

def produce_plots(manipulator, current_time, save=False):
    
    admissible_region = manipulator.admissible_region
    admissible_region_grid = [manipulator.s_lims, manipulator.sd_lims]
    s_axisq1 = manipulator.s_axisq1
    s_axisq2 = manipulator.s_axisq2
    coordinates_q1_q2 = manipulator.coordinates_q1_q2
    link_lengths = manipulator.link_lengths
    x_y_coordinates = manipulator.x_y_coordinates
                  
    plotter = two_dof_robot_data_visualisation(current_time)
    
    plotter.generate_state_space_plot(admissible_region,save, 1, "admissible_region", "numerical_example_1/")
    
    plotter.plot_q1_against_q2(coordinates_q1_q2,save,5,"joint_space", "numerical_example_1/")
    plotter.plot_robot_motion_x_y(link_lengths, x_y_coordinates, s_axisq1, save,5,"workspace","numerical_example_1/")
    
def produce_extreme_trajectory_plots(trajectory_list, current_time, save=False):

    
    plotter = two_dof_robot_data_visualisation(current_time)

    label_list = ['T_l', 'T_a']
    color_list = ['g', 'b']

    plotter.overlay_trajectories_with_admissible_region("N/A",\
                                                        trajectory_list,\
                                                        label_list,color_list,\
                                                        save,1,"numerical_example_1/",\
                                                        "extreme_trajectory_plot")

def produce_extreme_trajectory_plots_with_extra(T, trajectory_list, current_time, save=False):
    
    plotter = two_dof_robot_data_visualisation(current_time)

    label_list = ['T_l', 'T_a', 'T', 'T']
    color_list = ['g', 'b', 'r', 'r']

    trajectory_list.extend(T)
    #print(len(trajectory_list))
    
    plotter.overlay_trajectories_with_admissible_region("N/A",\
                                                        trajectory_list,\
                                                        label_list,color_list,\
                                                        save,1,"numerical_example_1/",\
                                                        "extreme_trajectory_plot")
        
      
def plot_multiple_trajectories(trajectory_list, current_time, folder, save=False ):
    
    plotter = two_dof_robot_data_visualisation(current_time)

    label_list = ['Td', 'Ta', 'T_constraint', 'upper_extreme','lower_exterme', 'Tlower', 'Tu1', 'Tu2', 'Tu3' ]
    color_list = ['orange', 'orange', 'darkgreen', 'orange', 'orange', 'orange', 'orange','orange', 'orange']

    T_len = len(trajectory_list)
    lab_len = len(label_list)
    col_len = len(color_list)
    #print("label list length ", lab_len)
    #print("color list length ", col_len)
    #print("trajectory list length ", T_len)
    
    if T_len > lab_len:
        diff = T_len - lab_len
        #print(diff)
        while diff > 0:
            label_list.append("T_extra")
            diff = T_len - len(label_list)
    
    if T_len > col_len:
        last_el = color_list[-1]
        diff = T_len - lab_len
        #print(diff)
        while diff >0:
            color_list.append(last_el)
            diff = T_len - len(color_list)            
   
    plotter.overlay_trajectories_with_admissible_region("N/A",\
                                                        trajectory_list,\
                                                        label_list,color_list,\
                                                        save,1,folder,\
                                                        "trajectory_plots", title="Region of Stablisability")
    
def plot_constraints(VLC, soft_constraints, current_time, save_file):
    
    plotter = two_dof_robot_data_visualisation(current_time)
    plotter.plot_limit_curves_paper(VLC, soft_constraints, save=save_file)
    


def plot_controller_sit_1(trajectory_list, current_time, save=False):
    
    plotter = two_dof_robot_data_visualisation(current_time)
    
    label_list = ['T', 'T', 'T', 'T','T', 'Tu1', 'Tu2', 'Tu3', 'Tu4' ]
    color_list = ['orange', 'orange', 'orange', 'orange', 'orange', 'orange', 'purple','purple','purple','purple', 'purple','black']

    T_len = len(trajectory_list)
    lab_len = len(label_list)
    col_len = len(color_list)
    #print("label list length ", lab_len)
    #print("color list length ", col_len)
    #print("trajectory list length ", T_len)
    
    if T_len > lab_len:
        diff = T_len - lab_len
        #print(diff)
        while diff > 0:
            label_list.append("T_extra")
            diff = T_len - len(label_list)
    if T_len > col_len:
        last_el = color_list[-1]
        diff = T_len - lab_len
        #print(diff)
        while diff >0:
            color_list.append(last_el)
            diff = T_len - len(color_list)            
    
    plotter.overlay_trajectories_with_admissible_region("N/A",\
                                                        trajectory_list,\
                                                        label_list,color_list,\
                                                        save,1,filepath="paper_plots/",\
                                                        filename="controller_plot", title="controller")
    
def plot_controller_sit_8_1(trajectory_list, current_time, boundary_list_len, save=False):
    
    plotter = two_dof_robot_data_visualisation(current_time)

    label_list = ['T', 'T', 'T', 'T','T', 'Tu1', 'Tu2', 'Tu3', 'Tu4' ]
    #color_list = ['orange', 'orange', 'orange', 'orange', 'orange', \
                  #'orange', 'orange','orange','orange','orange', \
                      #'orange','orange','orange','orange','orange','orange',\
                          #'purple', 'purple']

    boundary_color = 'orange'
    control_color = 'purple'
    color_list = []
    i=0

    while i < boundary_list_len:
        if i == 0:
            color_list = [boundary_color]
        else:
            color_list. append(boundary_color)
        i = i + 1
        
    color_list.append(control_color)
    
    T_len = len(trajectory_list)
    lab_len = len(label_list)
    col_len = len(color_list)
    #print("label list length ", lab_len)
    #print("color list length ", col_len)
    #print("trajectory list length ", T_len)
    
    if T_len > lab_len:
        diff = T_len - lab_len
        #print(diff)
        while diff > 0:
            label_list.append("T_extra")
            diff = T_len - len(label_list)

    if T_len > col_len:
        last_el = color_list[-1]
        diff = T_len - lab_len
        #print(diff)
        while diff >0:
            color_list.append(last_el)
            diff = T_len - len(color_list)            
    
    print(len(color_list))
    print(len(label_list))
    plotter.overlay_trajectories_with_admissible_region("N/A",\
                                                        trajectory_list,\
                                                        label_list,color_list,\
                                                        save,1,filepath="paper_plots/",\
                                                        filename="controller_plot_8_1", title="controller")
        
        
def plot_actuation_level_sit_8_1(list_to_plot, current_time, save=False):
    
    plotter = two_dof_robot_data_visualisation(current_time)
         
   
    plotter.plot_actuation_level_list(list_to_plot,\
                        save,\
                        filepath="paper_plots/",\
                        filename="actuation_level_8_1",\
                        title="lambda plot")
        
        
def plot_inputs_sit_8_1(list_to_plot, current_time, save=False):
    
    plotter = two_dof_robot_data_visualisation(current_time)
         
   
    plotter.plot_input_list(list_to_plot,\
                        save,\
                        filepath="paper_plots/",\
                        filename="input_plot_8_1",\
                        title="u(x)")
        
def plot_Ax_sit_8_1(list_to_plot, current_time, save=False):
    
    plotter = two_dof_robot_data_visualisation(current_time)
         
   
    plotter.plot_input_list(list_to_plot,\
                        save,\
                        filepath="paper_plots/",\
                        filename="A(x)_plot_8_1",\
                        title="A(x)")
        
def plot_Dx_sit_8_1(list_to_plot, current_time, save=False):
    
    plotter = two_dof_robot_data_visualisation(current_time)
         
   
    plotter.plot_input_list(list_to_plot,\
                        save,\
                        filepath="paper_plots/",\
                        filename="D(x)_plot_8_1",\
                        title="D(x)")        
        
        
def plot_input_vs_x_sit_8_1(list_to_plot, current_time, save=False):      
    
    plotter = two_dof_robot_data_visualisation(current_time)
         
    
    #print(list_to_plot)
    
    
    plotter.plot_actuation_level_vs_x(list_to_plot,\
                        save,\
                        filepath="paper_plots/",\
                        filename="input_level_8_1_vs_x1",\
                        title="u(x) vs x_1 plot",\
                        axis_label="u(x)") 
    

def plot_upper_constraint_vs_x_sit_8_1(list_to_plot, current_time, save=False):      
    
    plotter = two_dof_robot_data_visualisation(current_time)
         
    
    #print(list_to_plot)
    
    
    plotter.plot_actuation_level_vs_x(list_to_plot,\
                        save,\
                        filepath="paper_plots/",\
                        filename="upper_constraint_level_8_1_vs_x1",\
                        title="A(x) vs x_1 plot",\
                        axis_label="A(x)") 


def plot_lower_constraint_vs_x_sit_8_1(list_to_plot, current_time, save=False):      
    
    plotter = two_dof_robot_data_visualisation(current_time)
         
    
    #print(list_to_plot)
    
    
    plotter.plot_actuation_level_vs_x(list_to_plot,\
                        save,\
                        filepath="paper_plots/",\
                        filename="lower_constraint_level_8_1_vs_x1",\
                        title="D(x) vs x_1 plot",\
                        axis_label="D(x)") 

def plot_upper_constraint_vs_x_sit_8_1(list_to_plot, current_time, save=False):      
    
    plotter = two_dof_robot_data_visualisation(current_time)
         
    
    #print(list_to_plot)
    
    
    plotter.plot_actuation_level_vs_x(list_to_plot,\
                        save,\
                        filepath="paper_plots/",\
                        filename="upper_constraint_level_8_1_vs_x1",\
                        title="A(x) vs x_1 plot",\
                        axis_label="A(x)")
        
        
def plot_tau_1_vs_x_sit_8_1(list_to_plot, current_time, save=False):      
    
    plotter = two_dof_robot_data_visualisation(current_time)
         
    
    #print(list_to_plot)
    
    
    plotter.plot_actuation_level_vs_x(list_to_plot,\
                        save,\
                        filepath="paper_plots/",\
                        filename="tau_1_8_1_vs_x1",\
                        title="tau 1 vs x_1 plot",\
                        axis_label="$\tau_1$") 
        
def plot_tau_2_vs_x_sit_8_1(list_to_plot, current_time, save=False):      
    
    plotter = two_dof_robot_data_visualisation(current_time)
         
    
    #print(list_to_plot)
    
    
    plotter.plot_actuation_level_vs_x(list_to_plot,\
                        save,\
                        filepath="paper_plots/",\
                        filename="tau_2_8_1_vs_x1",\
                        title="tau 2 vs x_1 plot",\
                        axis_label="$\tau_2$")  

def plot_both_tau_vs_x_sit_8_1(list_to_plot, current_time, save=False):      
    
    plotter = two_dof_robot_data_visualisation(current_time)
         
    
    #print(list_to_plot)
    
    
    plotter.plot_two_vs_x1(list_to_plot,\
                        save,\
                        filepath="paper_plots/",\
                        filename="tau_2_8_1_vs_x1",\
                        title="torques vs x_1 plot",\
                        axis_label="$\tau _2$")  
    
def plot_actuation_level_vs_x_sit_8_1(list_to_plot, current_time, save=False):
    
    plotter = two_dof_robot_data_visualisation(current_time)
         
    
    #print(list_to_plot)
    
    
    plotter.plot_actuation_level_vs_x(list_to_plot,\
                        save,\
                        filepath="paper_plots/",\
                        filename="actuation_level_8_1_vs_x1",\
                        title="lambda plot") 
    
    #plotter.plot_actuation_level_list(list_to_plot,
    #                    save,\
    #                    filepath="paper_plots/",\
    #                    filename="actuation_level_8_1",\
    #                    title="lambda plot")
        
        
def plot_controller_sit_8_1_with_guides(trajectory_list, upper_guide, lower_guide,\
                                        current_time, boundary_list_len, save=False):

    plotter = two_dof_robot_data_visualisation(current_time)

    label_list = ['T', 'T', 'T', 'T','T', 'Tu1', 'Tu2', 'Tu3', 'Tu4' ]

    boundary_color = 'orange'
    control_color = 'purple'
    color_list = []
    i=0

    while i < boundary_list_len:
        if i == 0:
            color_list = [boundary_color]
        else:
            color_list. append(boundary_color)
        i = i + 1
        
    color_list.append(control_color)
    
    T_len = len(trajectory_list)
    lab_len = len(label_list)
    col_len = len(color_list)
    #print("label list length ", lab_len)
    #print("color list length ", col_len)
    #print("trajectory list length ", T_len)
    
    if T_len > lab_len:
        diff = T_len - lab_len
        #print(diff)
        while diff > 0:
            label_list.append("T_extra")
            diff = T_len - len(label_list)

    if T_len > col_len:
        last_el = color_list[-1]
        diff = T_len - lab_len
        #print(diff)
        while diff >0:
            color_list.append(last_el)
            diff = T_len - len(color_list)            
    
    print(len(color_list))
    print(len(label_list))
    plotter.overlay_trajectories_with_admissible_region_with_guides("N/A",\
                                                        trajectory_list,\
                                                        upper_guide,\
                                                        lower_guide,\
                                                        label_list,color_list,\
                                                        save,1,filepath="paper_plots/",\
                                                        filename="controller_plot_8_1", title="controller")        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        