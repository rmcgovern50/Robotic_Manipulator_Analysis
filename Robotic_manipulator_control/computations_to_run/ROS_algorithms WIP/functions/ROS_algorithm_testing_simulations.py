# -*- coding: utf-8 -*-
from robot_models import two_dof_planar_robot
from functions.controller import path_dynamics_controller
import numpy as np
import matplotlib.pyplot as plt
import my_math as mm
import save_data as save_data
from my_sorting import combine_to_tuples


def generate_point_to_point(robot_parameters, s_lim, sd_lim, line_definition, initial_coordinate, final_coordinate, current_time):
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

def perform_u_scan(manipulator, target_state):
    "function that scans the axis and shows the min input at L=1"
    control = path_dynamics_controller(manipulator)
    evaluated_list = control.scan_input_signs_on_x1_axis(0, target_state, number_of_steps=10, L=1)
    
    #ux = [x[0] for x in evaluated_list]
    #x1 = [x[1][0] for x in evaluated_list]
    #x2 = [x[1][1] for x in evaluated_list]
    
    #print("input value", ux)
    #print("x1 ", x1)
    #print("x2 ", x2)
    #print("min value ", min(ux))
    return evaluated_list   

def generate_a_trajectory(manipulator, start_state, L):
    
    control = path_dynamics_controller(manipulator)
    T, _, _ = control.simulate_trajectory(start_state, L, 1, 0.1)    
    
    return [T]
    


def Step_1(current_time, run_full_simulation=True):
    
    robot = {'joint_masses': [0.25, 0.25],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits': [(-10,10), (-10,10)]\
            }
    simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                              'x2_lim':  [0,20, 0.01],\
                              'line_definition': ["straight line", [(0.34,-0.20),(0.34, 0.20)]],\
                              'initial_state': (0,0),\
                              'final_state': (1,11.5)\
                              }
    if run_full_simulation:    
        
        manipulator = two_dof_planar_robot(robot, current_time)   
        manipulator.run_full_path_dynamics_analysis(simulation_parameters['line_definition'], \
                                                    simulation_parameters['x1_lim'], \
                                                    simulation_parameters['x2_lim'])
        
        #simulate the kinematic movements of the robot
        manipulator.simulate_trajectory(manipulator.qs, 0.1)
            
        save_data.save_obj(manipulator, "ROS data/", "step_1_manipulator")        
    else:
        manipulator = save_data.load_obj("ROS data/", "step_1_manipulator")  
    
    
    return manipulator
        
        
        
def Step_2(current_time, manipulator, run_curve_simulations=True,\
           run_constraint_simulations=True,\
           run_ROS_boundary_xd_sim=True, run_ROS_boundary_xa_sim=True,\
           run_extract_boundaries_from_extreme_trajectories = True):

    
    if run_curve_simulations:
        target_state = (0.9, 8.5)
        
        #produce an extreme trajectory set from the extreme trajectories
        extreme_trajectories = \
        generate_extreme_trajectories_to_target(manipulator, target_state)
        save_data.save_obj(extreme_trajectories, "ROS data/", "step_2_extreme_trajectories")    
        
    else:
        extreme_trajectories = save_data.load_obj("ROS data/", "step_2_extreme_trajectories")

    if run_constraint_simulations:
        
        #generate_arbitrary_constraint_set()
        soft_constraint_curve = generate_arbitrary_constraint_set_data()    
        #print(len(manipulator.boundary_points), len(soft_constraint_curve) )
        min_data = mm.min_function_two_constraints(manipulator.boundary_points, soft_constraint_curve)
        
        constraint_curve = [item for sublist in min_data for item in sublist]
        #constraint_curve = manipulator.boundary_points
        save_data.save_obj(constraint_curve, "ROS data/", "step_2_constraint_curve")        
        
    else:
        constraint_curve = save_data.load_obj("ROS data/", "step_2_constraint_curve") 

    #define the xa intersect and xd intersect    
    if run_ROS_boundary_xd_sim:
        xd_intersect = mm.find_intersect_in_trajectories(constraint_curve,\
                                              extreme_trajectories[0], plot=False) 
        if xd_intersect == False:             
            xd_intersect = extreme_trajectories[0][-1]
            
        save_data.save_obj(xd_intersect, "ROS data/", "step_2_xd")        
    else:
        xd_intersect = save_data.load_obj("ROS data/", "step_2_xd") 
        

    if run_ROS_boundary_xa_sim:
        xa_intersect = mm.find_intersect_in_trajectories(constraint_curve,\
                                              extreme_trajectories[1], plot=False)
        #print(extreme_trajectories[1])
        if xa_intersect == (False, False, False):             
            xa_intersect = extreme_trajectories[1][-1]

        save_data.save_obj(xa_intersect, "ROS data/", "step_2_xa")
    else:
        xa_intersect = save_data.load_obj("ROS data/", "step_2_xa") 
           
    #print("xd", xd_intersect)
    #print("xa", xa_intersect)    
    
    if run_extract_boundaries_from_extreme_trajectories:
        
        Td = extreme_trajectories[0]
        Ta = extreme_trajectories[1]        
        """
        loop through each list and stop when the intersection is found
        """
        i = 0
        while Td[i][0] > xd_intersect[0] and i < len(Td):
            i = i + 1
        last_index_Td = i 

        i=0
        while Ta[i][0] > xa_intersect[0] and i < len(Ta):
            i = i + 1
        last_index_Ta = i       

        boundaries_from_extreme_trajectories = [Td[0:last_index_Td], Ta[0:last_index_Ta]]
        save_data.save_obj(boundaries_from_extreme_trajectories, "ROS data/", "boundaries_from_extreme_trajectories")
    else:
        boundaries_from_extreme_trajectories = save_data.load_obj("ROS data/", "boundaries_from_extreme_trajectories") 
        
    
    
    return extreme_trajectories, boundaries_from_extreme_trajectories ,xa_intersect, xd_intersect, constraint_curve
        
def generate_arbitrary_constraint_set_data():

    x = np.linspace(0, 1, 1000)  
    #y = np.linspace(8, 8, 50)      
    #y = 4*np.sin(10*x) + 2*np.sin(18*x) + 10  
    #y =  4*np.sin(10*(x)**2) + -2*np.sin(18*x + x) + np.exp(-1*x) + x + 5*x**2 + 3
    y = (7*(x-0.5))**2 +9
    #y =  50*(-1*(x-0.5))**3 + 8
    #print(x)
    #print(y)
    z=np.polyfit(x,y, 20) #x and y describe function to be approximated, the number is the order of polynomial

    y_calc = np.polyval(z, x)
    
    trajectory = combine_to_tuples(x, y_calc)
    #print()
    
    #print(z)
    #print(z1, z)
    #plt.plot(x, y_calc, ms=1)
    #plt.plot(x, y, 'or')
    
    #plt.xlabel("s")
    #plt.ylabel("$\dot{s}$")
    #plt.ylim(ymin=0)
    #plt.show() 
    #print(len(trajectory))
    return trajectory
        
def Step_3(manipulator, xa_intersect, run_full_simulation=True):

    evaluated_inputs = perform_u_scan(manipulator, xa_intersect[0])
    ux = [x[0] for x in evaluated_inputs]
    
    min_input = min(ux)

    if run_full_simulation:        
        if min_input >= 0:
            print("it is all valid")
            x = list(np.linspace(0, float(xa_intersect[0]), 100))
            y = list(0*np.linspace(0, float(xa_intersect[0]),100))
            lower_boundary = combine_to_tuples(x,y)
            
        else:
            print("more work needed")
        save_data.save_obj(lower_boundary, "ROS data/", "step_3_lower_boundary")
    else:
        lower_boundary = save_data.load_obj("ROS data/", "step_3_lower_boundary") 
    lower_boundary_trajectories = lower_boundary
    
    return lower_boundary_trajectories 

def Step_4(manipulator, xd_intersect, \
           constraint_curve, run_boundary_sim_setup=True,\
           run_boundary_sim=True):

    """
    First loop along the constraint curve and calculate the tangent, 
    support vector and the direction a max deceleration curve will go
    Create a list that contains the state and inner product so that
    the regions can be defined
    """
    control = path_dynamics_controller(manipulator)
    
    
    if run_boundary_sim_setup:
        i = 0
        for state in constraint_curve:
            x1 = state[0]
            x2 = state[1]
            if x1 > xd_intersect[0]:
                break
            else:
                """
                work out the direction of the minimum deceleration curve at each 
                point along the constraint curve. 
                """
                D, A = control.calc_upper_lower_bound_values(state)
                if D <= A:
                    #print(D, A, state)
                    traj_dir = np.array([x2, D], dtype=np.float32)
                    """
                    approxmimate the gradient of the data on the constraint curve 
                    using 2 data points (the current and next point)
                    """
                    #print("state", state , "next state", constraint_curve[i+1])
                    next_state = constraint_curve[i+1]
                    next_x1 = next_state[0]
                    next_x2 = next_state[1]
                    delta_x1 = next_x1 - x1
                    delta_x2 = next_x2 - x2
                    
                    #calculate the gradient
                    approx_gradient = np.array([delta_x1, delta_x2], dtype=np.float32)
                    #rotate it by 90 degrees up the way
                    support_vector = np.array([-1*delta_x2, delta_x1], dtype=np.float32)
                    
                    inner_product = np.dot(traj_dir, support_vector)            

                    if i == 0:
                       relevant_constraint_curve_section = [state]
                       T_dir_list = [traj_dir]
                       support_vector_list = [support_vector]
                       inner_product_list_with_state = [(state, inner_product)]
                    else:
                       relevant_constraint_curve_section.append(state)
                       T_dir_list.append(traj_dir)            
                       support_vector_list.append(support_vector)
                       inner_product_list_with_state.append((state, inner_product))        
                    i = i + 1

        """
        unpack the information on the inner products and save all the points in
        which the region changes. 
        
        A positive number means f(x)>=0 which means the
        trajectory will go outside the constraint in the next time step 
        
        A negative number means f(x)<0 which means the trajectory will go inside
        the constraint in the next time step
        """
        #initialise variables
        i=0
        sign_change = True
        prev_inner_product = 1
        
        for state_inner_product in inner_product_list_with_state:
            
            #abstract the state and inner product 
            state = state_inner_product[0]
            inner_product = state_inner_product[1]
                        
            #check if the current state has a sign change included 
            if (inner_product >= 0 and prev_inner_product  < 0) or \
               (inner_product < 0 and prev_inner_product  >= 0):
                sign_change=True
            
            #create a list of relevant states
            if i == 0:
                region_boundaries = [state]
                sign_change = False
                #save what the sign of the first region is
            else:
                if sign_change:
                    region_boundaries.append(state)        
                    sign_change = False
                    
            #save previous inner product
            prev_inner_product = inner_product
            
            i = i + 1

        
        #save the sign of the last inner product for the next step
        inner_product = inner_product_list_with_state[1][-1]
        print(inner_product)
        if inner_product >= 0:
            start_sign = "+ve"
        else:
            start_sign = "-ve"
        
        
        region_boundaries.append(xd_intersect)  
        save_data.save_obj(region_boundaries, "ROS data/", "step_4_region_boundaries")
        save_data.save_obj(start_sign, "ROS data/", "step_4_start_sign")
    
    else:
        region_boundaries = save_data.load_obj("ROS data/", "step_4_region_boundaries")        
        start_sign = save_data.load_obj("ROS data/", "step_4_start_sign")
    
    """
    now that the region boundary states are extracted the algorithm show be
    applied between
    
    A start sign being +ve means from region_boundary[-1] any trajectory emanating 
    from the constraint curve must necessarily move outside the constraint
    
    This means that if the sign is -ve the  from region_boundary[-1] will push 
    inside the curve
    
    So, if the first curve is 
    
    """

    if run_boundary_sim:    

        current_region_sign = start_sign
        region_number = 1
        
        upper_boundary_trajectories = []
        """
        If the start sign is -ve then we can add the entire first segment to the
        solution
        """
        if start_sign == "-ve":
            x1_low = region_boundaries[-(region_number+1)][0]
            x1_high = region_boundaries[-(region_number)][0]
            segment = extract_constraint_curve_segment(constraint_curve, x1_low, x1_high)
            upper_boundary_trajectories.append(segment)
            region_number = 2
            current_region_sign = "+ve" # each time we change region the sign will change
            
        #loop through and complete upper trajectory
        next_region_number = region_number + 1    
        
        """
        The current_region_sign will always begin as +ve here The loop will perform
        a repetitive process to extract all the remaining regions
        """
        i = 0    
        while next_region_number <= len(region_boundaries):
            
            #set the boundary state as the last number
            boundary_state = region_boundaries[-region_number]
            next_boundary_state = region_boundaries[-(region_number + 1)]
            
            #slide down the line a small bit for the algorithm to work
            if i == 0:
                slid_down_slightly_state = (boundary_state[0], boundary_state[1] - 1)
            else:
                slid_down_slightly_state = (boundary_state[0], prev_T_boundary_intersect[1] - 1)

            #integrate until the next boundary
            T, _, reason = control.simulate_trajectory(slid_down_slightly_state, L=0,\
                                                    direction=-1, T=0.01, \
                                                    x1_lim=region_boundaries[-next_region_number][0])

            prev_T_boundary_intersect = T[-1]
            """
            if the region is a -ve then an intersection with the constraint boundary 
            may occur and we will have to add some points to the T trajectory to 
            ensure we have the whole boundary
            """
            if current_region_sign == "-ve":
                """
                The difference between the end of T and where the boundary is should
                be measured if its larger than some value we have intersected a constraint early
                """
    
                difference = abs(prev_T_boundary_intersect[0] - next_boundary_state[0])
                #print(prev_T_boundary_intersect, next_boundary_state)
                
                """
                Two possible reasons we didnt get to the boundary can be 
                opproached using the same logic
                """
                
                if reason == "L>U" or difference > 0.02:
                    
                    """
                    loop through each state on the constraint curve and extract the
                    relevant index values
                    """
                    j = 0
                    while constraint_curve[j][0] < next_boundary_state[0]:
                        j = j + 1
                    
                    constraint_curve_lower_bound_index = j
                    
                    j = constraint_curve_lower_bound_index                
                    while constraint_curve[j][0] <= boundary_state[0]:
                        j = j + 1
                    constraint_curve_upper_bound_index = j                
                    
                    """
                    loop through the trajectory to ensure it does not pass over the
                    constraint curve
                    extract the relevant region
                    """
                    j = 0
                    #while T[j][0] > constraint_curve[constraint_curve_upper_bound_index][0]:
                    #    j = j +1
                    #last_index_to_take_on_T = j-1
                        
                    
                    last_index_to_take_on_T = len(T) - 1
                    
                    #print("constrain index", constraint_curve_lower_bound_index, constraint_curve_upper_bound_index)
                    #print("T last", T[last_index_to_take_on_T], "con 1st", constraint_curve[constraint_curve_lower_bound_index], "con last", constraint_curve[constraint_curve_upper_bound_index])
                    
                    """
                    finally form the T for this part
                    """
                    T_constraint =  constraint_curve[constraint_curve_lower_bound_index:constraint_curve_upper_bound_index]
                    T_constraint.reverse()
                    #print(T_constraint)
                    
                    T = list(T[0:last_index_to_take_on_T])
                    T.extend(T_constraint)
                    prev_T_boundary_intersect = constraint_curve[constraint_curve_lower_bound_index]
                    
            region_number = region_number + 1
            next_region_number = region_number + 1
            i = i + 1
            upper_boundary_trajectories.append(T)
            if current_region_sign == "+ve":
                current_region_sign= "-ve"
            elif current_region_sign=="-ve":
                current_region_sign = "+ve"

        save_data.save_obj(upper_boundary_trajectories, "ROS data/", "step_4_upper_boundary_trajectories")            
    else:
        upper_boundary_trajectories = save_data.load_obj("ROS data/", "step_4_upper_boundary_trajectories")

    return upper_boundary_trajectories
    



def extract_constraint_curve_segment(constraint_curve, lower_x1_bound, upper_x1_bound):
    
    i=0
    segment = []
    for state in constraint_curve:
        x1 = state[0]
        #check if its in range
        if x1 >= lower_x1_bound and x1 <= upper_x1_bound:
            if i == 0:
                segment = [state]
            else:
                segment.append(state)
        elif x1 > upper_x1_bound:
            break
        
        i = i + 1
        
    return segment
    
    
    
    
    
    
    
    
    
    
    
    
    

