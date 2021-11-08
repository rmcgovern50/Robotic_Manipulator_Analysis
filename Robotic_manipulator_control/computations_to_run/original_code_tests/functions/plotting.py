# -*- coding: utf-8 -*-
"""
@author: Ryan
This file contains methods that produce plots
"""

from plots.robot_data_visualisation import two_dof_robot_data_visualisation


def produce_all_plots(manipulator, current_time):
    
    #simulation_data = manipulator.sim_data
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


def produce_all_plots2(manipulator, current_time):


    admissible_region = manipulator.admissible_region
    admissible_region_grid = [manipulator.s_lims, manipulator.sd_lims]
    s_axisq1 = manipulator.s_axisq1
    s_axisq2 = manipulator.s_axisq2
    coordinates_q1_q2 = manipulator.coordinates_q1_q2
    link_lengths = manipulator.link_lengths
    x_y_coordinates = manipulator.x_y_coordinates
    
    
    #class to produce plots of manipulator
    plotter = two_dof_robot_data_visualisation(current_time)
    
    plotter.generate_state_space_plot(admissible_region)
    
    plotter.plot_simulation_parameters(s_axisq1, s_axisq2,\
                                        coordinates_q1_q2, link_lengths,\
                                        x_y_coordinates,\
                                         [1, "q1 v s"], [1, "q2 v s"], [1, "q1 vs q2"], [1, "workspace motion"],\
                                           [1,  "end effector motion"], [0, 50, "robot_gif_motion"],\
                                            [1, "sub plots q1vq2, workspace"], save=False)  
    
    plotter.generate_state_space_plot(admissible_region,save=False,\
                                      marker_size=1, title="state space")



def produce_collision_energy_vs_angle_plot(angle_vs_energy, current_time, save1=True, save2=True):
    #class to produce plots of manipulator
    plotter = two_dof_robot_data_visualisation(current_time)
    plotter.generate_collision_energy_angle_plot(angle_vs_energy, save1)
    plotter.plot_max_collision_energy_vs_direction_s(angle_vs_energy, save2)
    
def produce_bounded_Ek_projection_on_s_sdot_plane(energy_list, Ek_bound, current_time):
        plotter = two_dof_robot_data_visualisation(current_time)
        
        new_energy_list = []
        i=0
        
        #loop through the list and save all values needed for plot
        for el in energy_list:
            #print(el[2])
            if el[2] < Ek_bound:
                if i == 0:
                    new_energy_list =  [el]
                    i=1
                else:
                    new_energy_list.append(el)
                    
        #print(new_energy_list)
        
        plotter.generate_state_space_plot(new_energy_list, False, 1, "admissible_Energy_bound_projection_plot", "admissible_Energy_bound_projection_plot/")

def produce_admissible_region_plot(admissible_region, current_time):
     plotter = two_dof_robot_data_visualisation(current_time)
     plotter.generate_state_space_plot(admissible_region,False)
        
def produce_control_trajectory_admissable_region_plot(admissible_region, control_trajectory, switching_points, z, x, current_time, save=False):
    
    plotter = two_dof_robot_data_visualisation(current_time)
    #plotter.generate_state_space_plot(control_trajectory, False, 1, "control_trajectory", "polynomial_based_control/")
    #plotter.generate_polynomial_control_algorithm_plot(z, x, \
    #                                    control_trajectory, switching_points, \
    #                                    save, 1, filepath="s_sdot_plane")
        
    plotter.generate_overlayed_polynomial_control_algorithm_plot(z, x, \
                                        admissible_region, control_trajectory, switching_points, \
                                        save, 1)
    
def produce_colored_region_plot(admissible_regions, current_time, save=False):
    plotter = two_dof_robot_data_visualisation(current_time)
    plotter.generate_state_space_plot_with_colour_markers_for_acceleration(admissible_regions, save)
    
def produce_3D_acceleration_bound_plot(regions, current_time, save=False):
    plotter = two_dof_robot_data_visualisation(current_time)
    
    plotter.s_sdot_sddo_limit_plot(regions, save)
    
    
    
    
    
    
    
    