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
from path_dynamics_control import path_dynamics_controller
from robot_data_visualisation import two_dof_robot_data_visualisation
import save_data
import math as m
import datetime as dt


def save_obj(obj, name ):
    with open('saved_data/'+ name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)
        
def load_obj(name ):
    with open('saved_data/' + name + '.pkl', 'rb') as f:
        return pickle.load(f)


def test_plot_state_space(run_full_simulation=False):
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots
    
    if run_full_simulation:
            
        #Specify robot parameters
        #instatiate class with the physical parameters
        joint_masses = [0.25, 0.25]
        link_lengths = [0.2,0.2]
        actuator_limits = [(-10,10), (-10,10)]

        manipulator = two_dof_planar_robot(joint_masses, link_lengths, actuator_limits, current_time)
        
    
        #define grid to analyse
        s_lim = [0, 1, 0.01]
        sd_lim = [0,20, 0.1]
        #straight_line_definition = [starting_joints, ending_joints]     
        line_definition = [(0.25, 0.15), (-0.25,0.1)]
        direction = manipulator.get_direction_unit_vector(line_definition)
        
        manipulator.run_full_path_dynamics_analysis(line_definition, s_lim, sd_lim)
        manipulator.end_effector_Jacobian()
    
        #simulate the kinematic movements of the robot
        manipulator.simulate_trajectory(manipulator.qs,0.1)
            
        initial_coordinate =  (0,0)
        final_coordinate = (1,0)
        Controller = path_dynamics_controller(manipulator)
        
        control_trajectory, switching_points = Controller.generate_time_optimal_trajectory(initial_coordinate, final_coordinate)
        
        energy_list1 = manipulator.get_potential_collision_energy(manipulator.J_linear, manipulator.Mqs, [direction[0],direction[1] ,0], control_trajectory)
        manipulator.set_trajectory_energy_list(energy_list1)
        
        energy_list2 = manipulator.get_potential_collision_energy(manipulator.J_linear, manipulator.Mqs, [direction[0],direction[1] ,0], manipulator.admissible_region)
        manipulator.set_admissible_region_collision_energies(energy_list2)        
        
        simulation_data = manipulator.set_simulation_data(control_trajectory, switching_points)
        #put in json file
        save_data.save_obj(simulation_data, "full_simulation_data/", "prev_sim_data")
        #save_all_data(manipulator, current_time, energy_list1)
    else:
        
        simulation_data = save_data.load_obj("full_simulation_data/", "prev_sim_data",)
        
        

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
    
    plotter.generate_state_space_plot(admissible_region, False)
    """
    plotter.plot_simulation_parameters(s_axisq1, s_axisq2,\
                                        coordinates_q1_q2, link_lengths,\
                                        x_y_coordinates,\
                                         [1, "q1 v s"], [1, "q2 v s"], [1, "q1 vs q2"], [1, "workspace motion"],\
                                           [1,  "end effector motion"], [1, 50, "robot_gif_motion"],\
                                            [1, "sub plots q1vq2, workspace"], True)  
    """

    
    #plotter.generate_control_algorithm_plot(admissible_region, control_trajectory, switching_points)
    
    #plotter.plot_potential_collision_energies_s(energy_list)
    #plotter.plot_potential_collision_energies_sdot(energy_list)
    #plotter.plot_potential_collision_energies_s_sdot(energy_list)
    
    #plotter.plot_potential_collision_energies_s_sdot_surf(admissible_region_collision_energy)

    print("done")


def save_all_data(manipulator, current_time, energy_list):
    save_data.export_tuple_list(manipulator.admissible_region, ("s", "s dot"), "admissible_region/" ,"admissible region " + current_time)
    save_data.export_tuple_list(manipulator.boundary_points, ("s", "s dot"), "admissible_region_boundary/", "admissible region boundary "  + current_time)
    save_data.export_tuple_list(manipulator.s_axisq1, ("s", "q1"), "q1_vs_s/" , "q1 vs s " + current_time)
    save_data.export_tuple_list(manipulator.s_axisq2, ("s", "q2"), "q2_vs_s/" ,"q2 vs s " + current_time)
    save_data.export_tuple_list(energy_list, ("s", "s dot", "E_k"), "potential_collision_energy/" ,"s plane trajectory and collision energy " + current_time)
        
if __name__ == "__main__": 
    
    test_plot_state_space(False)