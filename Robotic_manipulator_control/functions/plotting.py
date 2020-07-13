# -*- coding: utf-8 -*-
"""
@author: Ryan
This file contains methods that produce plots
"""

from plots.robot_data_visualisation import two_dof_robot_data_visualisation


def produce_all_plots(simulation_data, current_time):
    admissible_region = simulation_data['admissible_region']
    admissible_region_grid = simulation_data['admissible_region_grid']
    admissible_region_collision_energy = simulation_data['admissible_region_collision']
    s_axisq1 = simulation_data['q1_vs_s']
    s_axisq2 = simulation_data['q2_vs_s']
    coordinates_q1_q2 = simulation_data['q1_vs_q2']
    link_lengths = simulation_data['link_lengths']
    x_y_coordinates = simulation_data['xy_coordinates']
    control_trajectory = simulation_data['control_trajectory']
    energy_list = simulation_data['potential_collision_energy']
    switching_points = simulation_data['switching_points']
                  
                  
    #class to produce plots of manipulator
    plotter = two_dof_robot_data_visualisation(current_time)
    
    plotter.generate_state_space_plot(admissible_region)
    
    plotter.plot_simulation_parameters(s_axisq1, s_axisq2,\
                                        coordinates_q1_q2, link_lengths,\
                                        x_y_coordinates,\
                                         [1, "q1 v s"], [1, "q2 v s"], [1, "q1 vs q2"], [1, "workspace motion"],\
                                           [1,  "end effector motion"], [1, 50, "robot_gif_motion"],\
                                            [1, "sub plots q1vq2, workspace"], True)  
    
    plotter.generate_control_algorithm_plot(admissible_region, control_trajectory, switching_points)
    plotter.plot_potential_collision_energies_s(energy_list)
    plotter.plot_potential_collision_energies_sdot(energy_list)
    plotter.plot_potential_collision_energies_s_sdot(energy_list)
    plotter.plot_potential_collision_energies_s_sdot_surf(admissible_region_collision_energy)
    
def produce_collision_energy_vs_angle_plot(angle_vs_energy, current_time):
    #class to produce plots of manipulator
    plotter = two_dof_robot_data_visualisation(current_time)
    plotter.generate_collision_energy_angle_plot(angle_vs_energy)
    
def produce_bounded_Ek_projection_on_s_sdot_plane(energy_list, Ek_bound, current_time):
        plotter = two_dof_robot_data_visualisation(current_time)
        
        new_energy_list = []
        i=0
        
        #loop through the list and save all values needed for plot
        for el in energy_list:
            print(el[2])
            if el[2] < Ek_bound:
                if i == 0:
                    new_energy_list =  [el]
                    i=1
                else:
                    new_energy_list.append(el)
                    
        print(new_energy_list)
        
        plotter.generate_state_space_plot(new_energy_list, True, 1, "admissible_Energy_bound_projection_plot", "admissible_Energy_bound_projection_plot/")
        
        
        
        
        
        
        
        
    