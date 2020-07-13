# -*- coding: utf-8 -*-
from robot_models import two_dof_planar_robot
from path_dynamics_control import path_dynamics_controller
from math import pi
import my_math as mm
import numpy as np

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

    Controller = path_dynamics_controller(manipulator)
    
    control_trajectory, switching_points = Controller.generate_time_optimal_trajectory(initial_coordinate, final_coordinate)

    direction = manipulator.get_direction_unit_vector(line_definition)
        
    energy_list1 = manipulator.get_potential_collision_energy(manipulator.J_linear, manipulator.Mqs, [direction[0],direction[1] ,0], control_trajectory)
    manipulator.set_trajectory_energy_list(energy_list1)
    
    energy_list2 = manipulator.get_potential_collision_energy(manipulator.J_linear, manipulator.Mqs, [direction[0],direction[1] ,0], manipulator.admissible_region)
    manipulator.set_admissible_region_collision_energies(energy_list2)        
    
    simulation_data = manipulator.set_simulation_data(control_trajectory, switching_points)
    
    return simulation_data

def run_direction_varying_experiment(robot_parameters, simulation_parameters, data, current_time):
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
    return -
        list of maximum collision energies with rotation angle along a time optimal path
        [(angle1, max_energy1), ... ((angle1, max_energy1))]
    """
    
    
    control_trajectory = data['control_trajectory']
    s_lim = simulation_parameters['s_lim']
    sd_lim = simulation_parameters['sd_lim']
    line_def = simulation_parameters['line_definition']
    #print("control trajectory", control_trajectory)
    
    #Specify robot parameters
    #instatiate class with the physical parameters
    manipulator = two_dof_planar_robot(robot_parameters, current_time)
    
    manipulator.run_full_path_dynamics_analysis(line_def, s_lim, sd_lim)
    manipulator.end_effector_Jacobian()
    direction = manipulator.get_direction_unit_vector(line_def)
    direction.append(0)
    rotation_angle = 0
    
    spinning_direction = direction
    
    #loop that works out the maximum collision energy at each angle of collision
    while rotation_angle <= 2*pi:

        spinning_direction = mm.rotate_xy(direction, rotation_angle)
        energy_list = manipulator.get_potential_collision_energy(manipulator.J_linear, manipulator.Mqs, [spinning_direction[0],spinning_direction[1] ,0], control_trajectory)
        
        temp_energy_list = [x[2] for x in energy_list]
        temp_energy_list_array = np.array(temp_energy_list)
        
        index = np.argmax(temp_energy_list)
        max_energy = max(temp_energy_list)
        
        print(max_energy, temp_energy_list[index])
        if rotation_angle == 0:
            max_energies = [(rotation_angle , max_energy)]
        else:
            max_energies.append((rotation_angle , max_energy))
            
        rotation_angle = rotation_angle + pi/2
            
    return max_energies








