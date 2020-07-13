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

import functions.plotting as plotting
import functions.simulation as sim

import saved_data.save_data as save_data
import math as m
import datetime as dt


def straight_line_test(robot, simulation_parameters, run_full_simulation=False):
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots
    
    if run_full_simulation:
        
        simulation_data = sim.Time_optimal_full_simulation(robot,\
                                                  simulation_parameters['s_lim'],\
                                                  simulation_parameters['sd_lim'],\
                                                  simulation_parameters['line_definition'],\
                                                  simulation_parameters['initial_ssd_coordinate'],\
                                                  simulation_parameters['final_ssd_coordinate'],\
                                                  current_time)
        
        save_data.save_obj(simulation_data, "full_simulation_data/", "prev_sim_data")

    else:
        
        simulation_data = save_data.load_obj("full_simulation_data/", "prev_sim_data")
        
        
    plotting.produce_all_plots(simulation_data, current_time)

    print("done")

def rotating_impact_direction_test(robot, simulation_parameters, run_full_simulation=False):

    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots
    
    if run_full_simulation==True:

        data = save_data.load_obj("full_simulation_data/", "prev_sim_data",)
        angle_vs_energy = sim.run_direction_varying_experiment(robot, simulation_parameters, data, current_time)
        
        save_data.save_obj(angle_vs_energy, "rotating_impact_direction_test/", "prev_sim_data")

        print("experimental data", angle_vs_energy)

    else:
        angle_vs_energy = save_data.load_obj("rotating_impact_direction_test/", "prev_sim_data")
        print("yeeaaaaa", angle_vs_energy)

    plotting.produce_collision_energy_vs_angle_plot(angle_vs_energy, current_time)

def produce_admissible_region_energy_projection_test(run_full_simulation=False):
    
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots
    
    if run_full_simulation==True:
        straight_line_test(True)

    simulation_data = save_data.load_obj("full_simulation_data/", "prev_sim_data")
    admissible_region_collision = simulation_data['admissible_region_collision']
    plotting.produce_bounded_Ek_projection_on_s_sdot_plane(admissible_region_collision, 1000, current_time)


if __name__ == "__main__": 
    robot = {'joint_masses': [0.25, 0.25],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits':  [(-10,10), (-10,10)]\
            }
     
    simulation_parameters = { 's_lim':[0, 1, 0.01],\
                              'sd_lim':  [0,20, 0.1],\
                              'line_definition': [(0.25, 0.15), (0.25,-0.15)],\
                              'initial_ssd_coordinate': (0,0),\
                              'final_ssd_coordinate': (1,0)\
                              }    
    
    #straight_line_test(robot, simulation_parameters, True)
    rotating_impact_direction_test(robot, simulation_parameters,True)
    #produce_admissible_region_energy_projection_test(True)