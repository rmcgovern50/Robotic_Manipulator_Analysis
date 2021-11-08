# -*- coding: utf-8 -*-
from robot_models import two_dof_planar_robot
from functions.example_controller import path_dynamics_controller
import numpy as np
import matplotlib.pyplot as plt
import my_math as mm

#=====================example 1================================================
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
    manipulator.run_full_path_dynamics_analysis(line_definition, s_lim, sd_lim)
    #simulate the kinematic movements of the robot
    manipulator.simulate_trajectory(manipulator.qs, 0.1)
    
    return manipulator

def generate_extreme_trajectories_to_target(manipulator, target_state):
    """

    Parameters
    ----------
    manipulator : TYPE
        robot object
    target_state : TYPE
        target state
    Returns
    -------
        two extreme trajectories
    """
    control = path_dynamics_controller(manipulator)
    T_l, _, _ = control.simulate_trajectory(target_state, 0, -1, 0.01)
    T_a, _, _ = control.simulate_trajectory(target_state, 1, -1, 0.01)

    return [T_l, T_a]

def ex_1_find_magic_L(manipulator, initial_state):
    
    control = path_dynamics_controller(manipulator)
    
    T1, _, _ = control.simulate_trajectory(initial_state, 1, 1, 0.01)
    T2, _, _ = control.simulate_trajectory(initial_state, 0, 1, 0.01)  
    #midpoint (1 + 0) / 2 = 0.5
    T3, _, _ = control.simulate_trajectory(initial_state, 0.5, 1, 0.01)
    #midpoint (1 + 0.5) / 2 = 0.75
    T4, _, _ = control.simulate_trajectory(initial_state, 0.75, 1, 0.01)
    #midpoint (1 + 0.75) / 2 = 0.875
    T5, _, _ = control.simulate_trajectory(initial_state, 0.875, 1, 0.01)  
    #midpoint (0.75 + 0.875) / 2 = 0.875
    T6, _, _ = control.simulate_trajectory(initial_state, 0.8125, 1, 0.01)
    #midpoint (0.75 + 0.8215) / 2 = 0.875
    T7, _, _ = control.simulate_trajectory(initial_state, 0.78125, 1, 0.01) 
    
    bisection_trajectories = [T1, T2, T3, T4, T5, T6, T7]
    
    return bisection_trajectories
    
#====================example 2======================================================

def straight_line_motion_cartesian_space(robot_parameters, x1_lim, \
                                         x2_lim, line_definition, \
                                         initial_state, \
                                         final_state,
                                         current_time):
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
    x1_lim : list
        definition of x1 region to scan in simulation
        [x1min, x1max, increment]
    x2_lim : list
        definition of x2 region to scan in simulation
        [x2min, x2max, increment]
    line_definition : list of tuples
        start and end coordinate of the end effector
        [(x(0), y(0)), (x(1), x(1)]
    initial_state : tuple
        initial coordinate in s sdot region
        (x1, x2)
    final_state : final coordinate in s sdot region
        (x1, x2)

    Returns
    -------
    manipulator object
    """
    
    #create a manipulator with the given robot parameters
    manipulator = two_dof_planar_robot(robot_parameters, current_time)
    manipulator.run_full_path_dynamics_analysis(line_definition, x1_lim, x2_lim)
    #simulate the kinematic movements of the robot
    manipulator.simulate_trajectory(manipulator.qs,0.1)
    
    return manipulator


def ex_2_generate_extreme_trajectories_to_target(manipulator, target_state):
    """

    Parameters
    ----------
    manipulator : TYPE
        robot object
    target_state : TYPE
        target state
    Returns
    -------
        two extreme trajectories
    """
    control = path_dynamics_controller(manipulator)
    T_d, _, _ = control.simulate_trajectory(target_state, 0, -1, 0.01)
    T_a, _, _ = control.simulate_trajectory(target_state, 1, -1, 0.01)

    return [T_d, T_a]



def ex_2_find_magic_L(manipulator, initial_state):
    
    control = path_dynamics_controller(manipulator)
    
    T1, _, _ = control.simulate_trajectory(initial_state, 1, 1, 0.01)
    T2, _, _ = control.simulate_trajectory(initial_state, 0, 1, 0.01)  
    #midpoint (1 + 0) / 2 = 0.5
    T3, _, _ = control.simulate_trajectory(initial_state, 0.5, 1, 0.01)
    #midpoint (1 + 0.5) / 2 = 0.75
    T4, _, _ = control.simulate_trajectory(initial_state, 0.75, 1, 0.01)
    #midpoint (1 + 0.75) / 2 = 0.875
    T5, _, _ = control.simulate_trajectory(initial_state, 0.875, 1, 0.01)    
    #midpoint (0.875 + 0.75) / 2 = 0.875
    T6, _, _ = control.simulate_trajectory(initial_state, 0.8125, 1, 0.01)
    #midpoint (0.875 + 0.8125) / 2 = 0.875
    T7, _, _ = control.simulate_trajectory(initial_state, 0.84375, 1, 0.01)        
    bisection_trajectories = [T1, T2, T3, T4, T5, T6, T7]
    
    return bisection_trajectories

def ex_2_approx_roa(manipulator, extreme_trajectories):
         
    Td = extreme_trajectories[0]
    Ta = extreme_trajectories[1]
    
    control = path_dynamics_controller(manipulator)
    
    Tl, is_it = control.check_if_lower_bound_is_straight_line(extreme_trajectories)
    #print(Tl)
    Tu = control.find_upper_bound_roa_approx(extreme_trajectories)   
    
    
    return Tl, Tu

def ex_2_find_lower_bound_on_region_of_attraction(manipulator, extreme_trajectories):
    Td = extreme_trajectories[0]
    Ta = extreme_trajectories[1]
    
    control = path_dynamics_controller(manipulator)
    
    print("last_point T_d = ", Td[-1])
    print("last_point T_a = ", Ta[-1])
    
    #define the origin to work out future bisections
    X0= (0,0)
    T1, _, _ = control.simulate_trajectory(X0, 1, 1, 0.05)
    
    X1 = (0.35,0)
    T2, _, _ = control.simulate_trajectory(X1, 1, 1, 0.05)
    
    X2 = (0.525,0)
    T3, _, _ = control.simulate_trajectory(X2, 1, 1, 0.05)
    
    X3 = (0.6125,0)
    T4, _, _ = control.simulate_trajectory(X3, 1, 1, 0.05)    
    
    X4 = (0.65625,0)
    T5, _, _ = control.simulate_trajectory(X4, 1, 1, 0.05)    
    
    X5 = (0.678125,0)
    T6, _, _ = control.simulate_trajectory(X5, 1, 1, 0.05)    
    
    X6 = (0.6890625,0)
    T7, _, _ = control.simulate_trajectory(X6, 1, 1, 0.05)    
    
    """
    if control.find_intersect(T1, Ta) != False:
        print("Ta intersected with")
    elif control.find_intersect(T1, Td) != False:
        print("Td intersected with")
    else:
        print("neither intersected with")
    """
    
    return [T1, T2, T3, T4, T5, T6, T7]

def ex_2_find_upper_bound_on_region_of_attraction(manipulator, extreme_trajectories):
    print("apply technique here!!!!")

def auto_find_magic_L(manipulator, X0, extreme_trajectories):
    control = path_dynamics_controller(manipulator)
    T_magic, L_magic, bisection_trajectories, L_list = control.perform_bisection_with_L(X0, extreme_trajectories, 0.05)

    print("L_list ", L_list)
    print("magic L ", L_magic)
    print("number of bisection trajectories = ", len(bisection_trajectories))

    return bisection_trajectories


def test_intersection_finder(manipulator, X0, extreme_trajectories):
    print("in here")
    control = path_dynamics_controller(manipulator)
    Td = extreme_trajectories[0]
    Ta = extreme_trajectories[1]
    
    T1, _, _ = control.simulate_trajectory(X0, 0.5, 1, 0.05)    
    intersection = mm.find_intersect_in_trajectories(Ta, T1)
    print(intersection)
    return [T1]




def make_selection_with_varying_end_effector_mass(robot, sim_range_list, simulation_parameters, current_time):
    """
    Parameters
    ----------
    robot : dictionary describing the robot
        DESCRIPTION.
    sim_range_list : [start mass, end mass, number of manipulators to creat]
        DESCRIPTION.

    Returns
    -------
    manipulators : = [manipulator 1, manipulator 2, ..., manupulator N]
        list of manipulators with different parameters that follow the same path.

    """
    
    #function that simply taes in a robot and some ranges to simulate its end effector mass over
    print(sim_range_list)
    start = sim_range_list[0]
    stop = sim_range_list[1]
    number_of_points = sim_range_list[2]
    
    sim_points = np.linspace(start, stop, number_of_points)
    
    #set sim parameters
    line_definition = simulation_parameters['line_definition']
    x1_lim = simulation_parameters['x1_lim']
    x2_lim = simulation_parameters['x2_lim']
    
    #print(sim_points)

    manipulators = []
    for mass in sim_points:
        
        robot['joint_masses'][1] = mass
        print("simulation running, end effector mass is = ", robot['joint_masses'][1], " kg")
        
        #create a manipulator and analyse it
        manipulator = two_dof_planar_robot(robot, current_time)
        
        manipulator.run_full_path_dynamics_analysis(line_definition, \
                                                    x1_lim, x2_lim)
        
        #simulate the kinematic movements of the robot
        manipulator.simulate_trajectory(manipulator.qs, 0.1)
        
        if len(manipulators) == 0:
            manipulators = [manipulator]
        else:
            manipulators.append(manipulator)
        
    
    return manipulators



def shoot_trajectory_for_each_manipulator(manipulators):
    
    trajectory_list = []
    for manipulator in manipulators:
        
        control = path_dynamics_controller(manipulator)
        
        #define the origin to work out future bisections
        X0= (0.1,6)
        T1, _, _ = control.simulate_trajectory(X0, 1, 1, 0.05)
        
        if len(trajectory_list) == 0:
            trajectory_list = [T1]
        else:
            trajectory_list.append(T1)            
    
    return trajectory_list
    
    
    
    




