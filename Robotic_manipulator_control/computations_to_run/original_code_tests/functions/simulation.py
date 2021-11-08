# -*- coding: utf-8 -*-
from robot_models import two_dof_planar_robot
from path_dynamics_control import path_dynamics_controller
from math import pi
import my_math as mm
import numpy as np
import math as m
import sympy as sp
import matplotlib.pyplot as plt
from shapely.geometry import Point, shape, Polygon


def Time_optimal_full_simulation(robot_parameters, s_lim, sd_lim, line_definition, initial_coordinate, final_coordinate, current_time):
    """
    
    Parameters
    ----------
    joint_masses : list
        masses of each joint of robot
        [m1, m2]
    link_lengths : list
        lengths of each link of the robot
        [l1, l2]
    actuator_limits : list of tuples
        limits on actuator torque
        [(t1min, t1max), (t2min, t2max)]
    s_lim : list
        definition of s region to scan in simulation
        [smin, smax, increment]
    sd_lim : list
        definition of s region to scan in simulation
        [sdmin, sdmax, increment]
    line_definition : list of tuples
        start and end coordinate of the end effector
        [(x1, y1), (x2, y2)]
    initial_coordinate : tuple
        initial coordinate in s sdot region
        (s1, sd1)
    final_coordinate : final coordinate in s sdot region
        (s2, sd2)

    Returns
    -------
    simulation_data - dictionary of data from this simulation

    """    
    
    #Specify robot parameters
    #instatiate class with the physical parameters
    manipulator = two_dof_planar_robot(robot_parameters, current_time)

    manipulator.run_full_path_dynamics_analysis(line_definition, s_lim, sd_lim)
    manipulator.end_effector_Jacobian()

    #simulate the kinematic movements of the robot
    manipulator.simulate_trajectory(manipulator.qs,0.1)

    #Controller = path_dynamics_controller(manipulator)
    
    #control_trajectory, switching_points = Controller.generate_time_optimal_trajectory(initial_coordinate, final_coordinate)

    direction = manipulator.get_direction_unit_vector(line_definition)
        
    #energy_list1 = manipulator.get_potential_collision_energy(manipulator.J_linear, manipulator.Mqs, [direction[0],direction[1] ,0], control_trajectory)
    #manipulator.set_trajectory_energy_list(energy_list1)
    
    energy_list2 = manipulator.get_potential_collision_energy(manipulator.J_linear, manipulator.Mqs, [direction[0],direction[1] ,0], manipulator.admissible_region)
    manipulator.set_admissible_region_collision_energies(energy_list2)        
    
    #simulation_data = manipulator.set_simulation_data(control_trajectory, switching_points)
    
    return manipulator


def generate_point_to_point_joint_space(robot_parameters, s_lim, sd_lim, line_definition, initial_coordinate, final_coordinate, current_time):
    """
    
    Parameters
    ----------
    joint_masses : list
        masses of each joint of robot
        [m1, m2]
    link_lengths : list
        lengths of each link of the robot
        [l1, l2]
    actuator_limits : list of tuples
        limits on actuator torque
        [(t1min, t1max), (t2min, t2max)]
    s_lim : list
        definition of s region to scan in simulation
        [smin, smax, increment]
    sd_lim : list
        definition of s region to scan in simulation
        [sdmin, sdmax, increment]
    line_definition : list of tuples
        start and end coordinate of the end effector
        [(q1(0), q2(0)), (q1(1), q2(1)]
    initial_coordinate : tuple
        initial coordinate in s sdot region
        (s1, sd1)
    final_coordinate : final coordinate in s sdot region
        (s2, sd2)

    Returns
    -------
    simulation_data - dictionary of data from this simulation

    """
    #create a manipulator with the given robot parameters
    manipulator = two_dof_planar_robot(robot_parameters, current_time)
    manipulator.run_full_path_dynamics_analysis(line_definition, s_lim, sd_lim, True)
    
    
    
    print("hello", line_definition)
 

def run_direction_varying_experiment(robot_parameters, simulation_parameters, data, current_time, manipulator):
    """
    Function which takes in dictionaries of the robot description and simulation
    and rotates the direction vector in the simulation, it uses the data produced by a previous
    simulation to extract a control trajectory
    
    Arguments - 

        robot = {'joint_masses': [0.25, 0.25],\
                 'link_lengths': [0.2,0.2],\
                 'actuator_limits':  [(-10,10), (-10,10)]\
                } 

           simulation_parameters = { 's_lim':[0, 1, 0.01],\
                                  'sd_lim':  [0,20, 0.1],\
                                  'line_definition': [(0.25, 0.15), (-0.25,0.1)],\
                                  'initial_ssd_coordinate': (0,0),\
                                  'final_ssd_coordinate': (1,0)\
                                  }
             data
    return -
        list of maximum collision energies with rotation angle along a time optimal path
        [(s1, sd1, angle1, max_energy1), ... ((s1, sd1, anglen, max_energyn))]
    """
    
    
    control_trajectory = data['control_trajectory']
    #print(control_trajectory)
    s_lim = simulation_parameters['s_lim']
    sd_lim = simulation_parameters['sd_lim']
    line_def = simulation_parameters['line_definition']
    #print("control trajectory", control_trajectory)
    

    direction = manipulator.get_direction_unit_vector(line_def)
    direction.append(0)
    rotation_angle = 0
    
    spinning_direction = direction
    
    #==========================================

    Jqs = manipulator.calc_qs_Jacobian(manipulator.J_linear,  manipulator.qs, manipulator.qds, manipulator.qdds)

    #==========================================
    
    #loop that works out the maximum collision energy at each angle of collision
    while rotation_angle <= 2*pi:

        spinning_direction = mm.rotate_xy(direction, rotation_angle)      

        energy_list = manipulator.get_potential_collision_energy(Jqs, manipulator.Mqs, [spinning_direction[0],spinning_direction[1] ,0], control_trajectory)

        
        temp_energy_list = [x[2] for x in energy_list]
        temp_energy_list_array = np.array(temp_energy_list)
        
        index = np.argmax(temp_energy_list)
        max_energy = max(temp_energy_list)

        if rotation_angle == 0:
            #list of 4 tuples of form (s, sdot, angle, energy)
            max_energies = [(energy_list[index][0], energy_list[index][1],rotation_angle , max_energy)]
        else:
            max_energies.append((energy_list[index][0], energy_list[index][1],rotation_angle , max_energy))
        
        rotation_angle = rotation_angle + pi/100
        
    return max_energies

def construct_admissible_region_description(manipulator):
    z, x = manipulator.convert_admissible_region_boundry_to_polynomial()
    return z, x

def generate_arbitrary_constraint_set():


    x = np.linspace(0, 1, 50)  
    #y = np.linspace(8, 8, 50)      
    #y = 4*np.sin(10*x) + 2*np.sin(18*x) + 10  
    #y =  4*np.sin(10*(x)**2) + -2*np.sin(18*x + x) + np.exp(-1*x) + x + 5*x**2 + 3
    y = (7*(x-0.25))**2 +10
    #y =  50*(-1*(x-0.5))**3 + 8
    #print(x)
    #print(y)
    z=np.polyfit(x,y, 20) #x and y describe function to be approximated, the number is the order of polynomial

    y_calc = np.polyval(z, x)
    print(z)
    #print(z1, z)
    plt.plot(x, y_calc, ms=1)
    plt.plot(x, y, 'or')
    
    plt.xlabel("s")
    plt.ylabel("$\dot{s}$")
    plt.ylim(ymin=0)
    plt.show()
    return z, x


def construct_control_trajectory(manipulator, z, x,\
                                s_start = 0, sd_start = 0,\
                                s_end = 1, sd_end  = 0,\
                                integration_step = 0.5,\
                                min_arc_length = 10,\
                                binary_search_multiplier = 1):

    Controller = path_dynamics_controller(manipulator)
    control_trajectory, switching_points, path_complete = Controller.polynomial_constraint_based_time_optimal_control(z, x,\
                                                         s_start, sd_start,\
                                                         s_end, sd_end,\
                                                         integration_step,\
                                                         min_arc_length,\
                                                         binary_search_multiplier)
        
    return control_trajectory, switching_points, path_complete


def form_polygon_regions(regions, plot=False):
    """
    Form the polygon objects for each of the regions
    experimental atm
    
    """


    #print(r)
    #print(len(r))
    p1 = Polygon(regions[0]) 
    p2 = Polygon(regions[1])
    p3 = Polygon(regions[2]) 
    p4 = Polygon(regions[3]) 
    #print(list(P.exterior.coords))
    #p2 = Polygon([(0.1, 2), (0.15, 8), (0.5, 10),(0.3,5), (0.6,1)])
    
    #p3 = P.intersection(p2)
    
    
    point_2_check = Point(0.16,7.5)
    xs = [point_2_check.x]
    ys = [point_2_check.y]
    plt.plot(xs, ys, 'or',color='g')

    
    in_P = p2.contains(point_2_check)
    
    if plot == True:
        
        x1 = [x[0] for x in list(p1.exterior.coords)]
        y1 = [x[1] for x in list(p1.exterior.coords)]
        #print(len(list(p3.exterior.coords)))
        plt.plot(x1, y1, color='r')
        
        x2 = [x[0] for x in list(p2.exterior.coords)]
        y2 = [x[1] for x in list(p2.exterior.coords)]
        #print(len(list(p3.exterior.coords)))
        plt.plot(x2, y2, color='g')
    
        x3 = [x[0] for x in list(p3.exterior.coords)]
        y3 = [x[1] for x in list(p3.exterior.coords)]
        #print(len(list(p3.exterior.coords)))
        plt.plot(x3, y3, color='c')
    
        x4 = [x[0] for x in list(p4.exterior.coords)]
        y4 = [x[1] for x in list(p4.exterior.coords)]
        #print(len(list(p3.exterior.coords)))
        plt.plot(x4, y4, color='c')
        
        plt.show() 
        
    return p1, p2, p3, p4

def run_backwards_reachibility(manipulator, polygon_list):
    
    Controller = path_dynamics_controller(manipulator)
    end_point = (1,10)
    Controller.backward_reachability_from_point(end_point, polygon_list)
    #form_polygon_regions(manipulator, regions)
    print("got here")
        
        
        
        
        
def run_full_simulation(robot_parameters, sim_parameters, current_time):


    #Specify robot parameters
    #instatiate class with the physical parameters
    manipulator = two_dof_planar_robot(robot_parameters, current_time)
    path_def = sim_parameters['path_def']
    x1_lim = sim_parameters["s_lim"]
    x2_lim = sim_parameters["sd_lim"]
    
    manipulator.run_full_path_dynamics_analysis(path_def, x1_lim, x2_lim)
    manipulator.end_effector_Jacobian()

    #simulate the kinematic movements of the robot
    manipulator.simulate_trajectory(manipulator.qs,0.1)

    
    return manipulator
     
        
        
        
        
        
        
        
        
        
        
        
        