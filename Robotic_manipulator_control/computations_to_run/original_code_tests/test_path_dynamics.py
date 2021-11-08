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
import sympy

import saved_data.save_data as save_data
import math as m
import datetime as dt
import matplotlib.pyplot as plt

def straight_line_test(robot, simulation_parameters, run_full_simulation=False):
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots
    
    if run_full_simulation:
        
        manipulator = sim.Time_optimal_full_simulation(robot,\
                                                  simulation_parameters['s_lim'],\
                                                  simulation_parameters['sd_lim'],\
                                                  simulation_parameters['line_definition'],\
                                                  simulation_parameters['initial_ssd_coordinate'],\
                                                  simulation_parameters['final_ssd_coordinate'],\
                                                  current_time)
        save_data.save_obj(manipulator, "manipulator_object/", "manipulator")
    else:
        manipulator = save_data.load_obj("manipulator_object/", "manipulator")        
        
    #plotting.produce_all_plots(manipulator, current_time)

    print("done")
    
def rotating_impact_direction_test(robot, simulation_parameters, run_full_simulation=False):

    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots
    
    if run_full_simulation==True:

        data = save_data.load_obj("full_simulation_data/", "prev_sim_data",)
        manipulator = save_data.load_obj( "manipulator_object/", "manipulator")
        angle_vs_energy = sim.run_direction_varying_experiment(robot, simulation_parameters, data, current_time, manipulator)
        
        save_data.save_obj(angle_vs_energy, "rotating_impact_direction_test/", "prev_sim_data")

        print("experimental data", angle_vs_energy)
    else:
        angle_vs_energy = save_data.load_obj("rotating_impact_direction_test/", "prev_sim_data")
        print("yeeaaaaa", angle_vs_energy)

    plotting.produce_collision_energy_vs_angle_plot(angle_vs_energy, current_time,False, False)

def produce_admissible_region_energy_projection_test(robot, simulation_parameters, run_full_simulation=False):
    
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots
    
    if run_full_simulation==True:
        straight_line_test(robot, simulation_parameters, True)

    simulation_data = save_data.load_obj("full_simulation_data/", "prev_sim_data")
    admissible_region_collision = simulation_data['admissible_region_collision']
    plotting.produce_bounded_Ek_projection_on_s_sdot_plane(admissible_region_collision, 1000, current_time)

def produce_boundary_polynomial_description(robot, simulation_parameters, run_full_simulation=False):
       
    if run_full_simulation:
        straight_line_test(robot, simulation_parameters, True)
        
    manipulator = save_data.load_obj( "manipulator_object/", "manipulator")    
    z, x = sim.construct_admissible_region_description(manipulator) 
    save_data.save_obj([z, x], "approximated_boundary_polynomial/", "z_x")


def generate_control_trajectory_based_on_single_polynomial_boundary(robot, simulation_parameters, run_full_simulation=False, find_control_trajectory=False):
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots
    if run_full_simulation==True:
        produce_boundary_polynomial_description(robot, simulation_parameters, True)

    manipulator = save_data.load_obj( "manipulator_object/", "manipulator")
    z, x = sim.generate_arbitrary_constraint_set()
    
    if find_control_trajectory == True:
        #approximate_region = save_data.load_obj( "approximated_boundary_polynomial/", "z_x")
        #z = approximate_region[0]
        #x = approximate_region[1]
        z, x = construct_constraint_curve_polynomial(robot)
        control_trajectory, switching_points, _ = sim.construct_control_trajectory(manipulator, z,x,\
                                                                                0.1,  7,\
                                                                                0.8,  3,\
                                                                                0.5,\
                                                                                  1 ,\
                                                                                   3 )
        obj = [control_trajectory, switching_points, z, x]
        save_data.save_obj(obj, "control_trajectory_with_limiting_polynomial/", "ct sw z x cubic 1")
    else:
        obj = save_data.load_obj("control_trajectory_with_limiting_polynomial/", "ct sw z x sine 1")
        control_trajectory = obj[0]
        switching_points = obj[1]
        z = obj[2]
        x = obj[3]
        
    #print(control_trajectory)
    plotting.produce_control_trajectory_admissable_region_plot(manipulator.admissible_region, control_trajectory, switching_points,\
                                                               z, x,\
                                                                current_time, False)
        
        
        
def construct_constraint_curve_polynomial(robot, max_energy = 15000):
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots        
    manipulator = save_data.load_obj( "manipulator_object/", "manipulator")
    #print(manipulator.admissible_region)
    actuator_constraints = manipulator.admissible_region_collision_energies

    #print(actuator_constraints)
    collision_energy_list= manipulator.create_energy_limit_projection(actuator_constraints, max_energy)
    #print(collision_energy_list)
    collision_energy_boundary = manipulator.create_boundary_description(collision_energy_list)
    #print(collision_energy_boundary)
    z, x = manipulator.convert_admissible_region_boundry_to_polynomial(collision_energy_boundary,False)
    
    #plotting.produce_admissible_region_plot(collision_energy_list, current_time)
    return z, x


def analyse_admissible_region_accelerations_by_region(run_sorting_algorithm=False):
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots
    manipulator = save_data.load_obj( "manipulator_object/", "manipulator")        
    
    if run_sorting_algorithm == True:

        sorted_admissible_region, sorted_admissible_region_3D = manipulator.get_admissible_region_sorted_by_acceleration_bounds()
        save_data.save_obj(sorted_admissible_region, "admissible_regions/", "region_sorted_acceleration")
        save_data.save_obj(sorted_admissible_region_3D, "admissible_regions/", "region_sorted_acceleration_3D")        
        #boundary_points_1 = manipulator.check_if_list_is_made_of_multiple_sets(sorted_admissible_region[0])
        #boundary_points_2 = manipulator.check_if_list_is_made_of_multiple_sets(sorted_admissible_region[1])
        #boundary_points_3 = manipulator.check_if_list_is_made_of_multiple_sets(sorted_admissible_region[2])
        #boundary_points_4 = manipulator.check_if_list_is_made_of_multiple_sets(sorted_admissible_region[3])        
        #save_data.save_obj(boundary_points_1, "admissible_regions/", "region_sorted_acceleration_boundary_1")
        #save_data.save_obj(boundary_points_2, "admissible_regions/", "region_sorted_acceleration_boundary_2")
        #save_data.save_obj(boundary_points_3, "admissible_regions/", "region_sorted_acceleration_boundary_3")          
        #save_data.save_obj(boundary_points_4, "admissible_regions/", "region_sorted_acceleration_boundary_4")
        
        

        
    else:
        sorted_admissible_region = save_data.load_obj("admissible_regions/", "region_sorted_acceleration")
        sorted_admissible_region_3D = save_data.load_obj("admissible_regions/", "region_sorted_acceleration_3D")        
        boundary_points_1 = save_data.load_obj("admissible_regions/", "region_sorted_acceleration_boundary_1")
        boundary_points_2 = save_data.load_obj("admissible_regions/", "region_sorted_acceleration_boundary_2")
        #boundary_points_3 = save_data.load_obj("admissible_regions/", "region_sorted_acceleration_boundary_3")
        boundary_points_4 = save_data.load_obj("admissible_regions/", "region_sorted_acceleration_boundary_4")
        
        sorted_points_1, _ = manipulator.sort_boundary(boundary_points_1, True)
        sorted_points_2, _ = manipulator.sort_boundary(boundary_points_2, True)
        #sorted_points_3, _ = manipulator.sort_boundary(boundary_points_3, True)
        sorted_points_4, other_region = manipulator.sort_boundary(boundary_points_4, True)        
        sorted_points_5, _ = manipulator.sort_boundary(other_region, True)    
        
        save_data.save_obj(sorted_points_1, "admissible_regions/", "ordered_region_sorted_acceleration_boundary_1")
        save_data.save_obj(sorted_points_2, "admissible_regions/", "ordered_region_sorted_acceleration_boundary_2")
        #save_data.save_obj(sorted_points_3, "admissible_regions/", "ordered_region_sorted_acceleration_boundary_3")          
        save_data.save_obj(sorted_points_4, "admissible_regions/", "ordered_region_sorted_acceleration_boundary_4")
        save_data.save_obj(sorted_points_5, "admissible_regions/", "ordered_region_sorted_acceleration_boundary_5")        
        #print(len(sorted_admissible_region))

        #print(len(boundary_points), len(sorted_admissible_region[1]))
    #print(sorted_admissible_region_3D[0])
    #sorted_points = manipulator.sort_boundary(boundary_points_4, True)
    #sim.form_polygon_regions(manipulator, sorted_points[1])
    
    
    #plotting.produce_colored_region_plot(sorted_admissible_region, current_time, False)
    #plotting.produce_3D_acceleration_bound_plot(sorted_admissible_region_3D, current_time, False)


def get_bounds_expressions():
    manipulator = save_data.load_obj( "manipulator_object/", "manipulator")
    manipulator.get_region_acceleration_bound_equations()


def test_create_shapely_polygons_for_regions(plot=False):
    sorted_points_1 = save_data.load_obj("admissible_regions/", "ordered_region_sorted_acceleration_boundary_1")
    sorted_points_2 = save_data.load_obj("admissible_regions/", "ordered_region_sorted_acceleration_boundary_2")
    #sorted_points_3 = save_data.load_obj("admissible_regions/", "ordered_region_sorted_acceleration_boundary_3")
    sorted_points_4 = save_data.load_obj("admissible_regions/", "ordered_region_sorted_acceleration_boundary_4")    
    sorted_points_5 = save_data.load_obj("admissible_regions/", "ordered_region_sorted_acceleration_boundary_5")
    
    regions = [sorted_points_1, sorted_points_2, sorted_points_4, sorted_points_5] 
    p1, p2, p3, p4 = sim.form_polygon_regions(regions, plot)
    
    return p1, p2, p3, p4
    

def run_reachability_analysis():
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots
    
    manipulator = save_data.load_obj( "manipulator_object/", "manipulator")
    sorted_admissible_region_3D = save_data.load_obj("admissible_regions/", "region_sorted_acceleration_3D")       
    p1, p2, p3, p4 = test_create_shapely_polygons_for_regions(True)
    polygon_list = [p1, p2, p3, p4]
    sim.run_backwards_reachibility(manipulator, polygon_list)




def movement_simulation_testing(robot, run_full_simulation=False, test_type="circular arc"):
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots


    if test_type=="circular arc":
        simulation_parameters = { 's_lim':[0, 1, 0.01],\
                                  'sd_lim':  [0,30, 0.1],\
                                  'path_def': ["circular arc", [(0, 0.3), 0.1]],\
                                  'initial_ssd_coordinate': (0,0),\
                                  'final_ssd_coordinate': (1,0)\
                                  }
    elif test_type=="straight line":
        simulation_parameters = { 's_lim':[0, 1, 0.01],\
                                  'sd_lim':  [0,30, 0.1],\
                                  'path_def': ["straight line", [(0.25, 0.3), (-0.15, 0.3)]],\
                                  'initial_ssd_coordinate': (0,0),\
                                  'final_ssd_coordinate': (1,0)\
                                  }        
    
    elif test_type == "joint space line":
        simulation_parameters = { 's_lim':[0, 1, 0.01],\
                                  'sd_lim':  [0,30, 0.1],\
                                  'path_def': ["joint space line", [(0, m.pi/2), (-m.pi/2, m.pi/2)]],\
                                  'initial_ssd_coordinate': (0,0),\
                                  'final_ssd_coordinate': (1,0)\
                                  }          
    else:
        run_full_simulation=False
        print("no valid test type input, simualtion not run")
            
    if run_full_simulation:
        
        manipulator = sim.run_full_simulation(robot,\
                                            simulation_parameters,\
                                            current_time)
            
        save_data.save_obj(manipulator, "manipulator_object/", "manipulator_circular_path")
    else:
        manipulator = save_data.load_obj("manipulator_object/", "manipulator_circular_path")        
    print("done")
    #plotting.produce_all_plots2(manipulator, current_time)




if __name__ == "__main__": 
    
    s, sd, sdd = sympy.symbols('s sd sdd')
    robot = {'joint_masses': [0.25, 0.25],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits':  [(-10/sd, 10/sd), (-10/sd, 10/sd)]\
            }
    #'actuator_limits':  [(-10, 10), (-10, 10)]\        
    #'actuator_limits':  [(-10/sd, 10/sd), (-10/sd, 10/sd)]\     
    simulation_parameters = { 's_lim':[0, 1, 0.01],\
                              'sd_lim':  [0,30, 0.1],\
                              'line_definition': [(0.25, 0.15), (-0.25,0.1)],\
                              'initial_ssd_coordinate': (0,0),\
                              'final_ssd_coordinate': (1,0)\
                              }
    print("running...")
    #straight_line_test(robot, simulation_parameters, True)
    #rotating_impact_direction_test(robot, simulation_parameters,False)
    #produce_admissible_region_energy_projection_test(robot, simulation_parameters, False)
    #produce_boundary_polynomial_description(robot, simulation_parameters, False)
    #generate_control_trajectory_based_on_single_polynomial_boundary(robot, simulation_parameters, False, True) 
    #construct_constraint_curve_polynomial(robot)
    #analyse_admissible_region_accelerations_by_region(False)
    #get_bounds_expressions()
    #test_create_shapely_polygons_for_regions(True)
    #run_reachability_analysis()
    #movement_simulation_testing(robot, run_full_simulation=True, test_type="circular arc")
    
    print("complete")