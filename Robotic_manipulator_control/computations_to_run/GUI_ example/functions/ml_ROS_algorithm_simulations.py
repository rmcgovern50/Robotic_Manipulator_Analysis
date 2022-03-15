# -*- coding: utf-8 -*-
from ml_robot_models import model
from functions.ml_controller import path_dynamics_controller
import numpy as np
import my_math as mm
import save_data as save_data
from my_sorting import combine_to_tuples
import sympy
from sympy import symbols
import warnings

import functions.ml_ROS_simulations_plotting as plotting



def Step_1(current_time,\
           simulation_parameters,\
           eng,\
           run_full_simulation=True,\
           save_folder="ROS_data/"):

        
    manipulator = model(eng, simulation_parameters, current_time)

    if run_full_simulation:
        manipulator.run_full_path_dynamics_analysis()
        stuff2pickle = manipulator.return_sim_reults()
        save_data.save_obj(stuff2pickle, save_folder, "step_1_manipulator")
        
    else:
        pickledStuff = save_data.load_obj(save_folder, "step_1_manipulator")  
        manipulator.set_sim_results(pickledStuff)
        
        
    return manipulator

def Step_2(manipulator, \
           target_interval, \
           situation, control, \
           run_curve_simulations=True, \
           work_out_steps_to_run=True, \
           run_constraint_simulations=True,\
           save_folder = "default"):
    
    #control = path_dynamics_controller(manipulator)
    #new_constraint_list = form_constraints_list(situation)    
    #control.set_constraint_list(new_constraint_list)
    
    if run_constraint_simulations:
        constraint_curve = control.min_of_constraints()
        save_data.save_obj(constraint_curve, save_folder, "step_2_constraint_curve")        
        
    else:
        constraint_curve = save_data.load_obj(save_folder, "step_2_constraint_curve")     
    
    if run_curve_simulations:
        #produce an extreme trajectory set from the extreme trajectories
        #control = path_dynamics_controller(manipulator)
        
        if isinstance(target_interval,list):
            print("we have an interval target")
            T_d, _, reason_Td = control.simulate_trajectory(target_interval[0], 0, -1, 0.05)
            T_a, _, reason_Ta = control.simulate_trajectory(target_interval[1], 1, -1, 0.05)
            print(reason_Td, reason_Ta)
        else:
            #print("we have a single state target")
            T_d, _, reason_Td = control.simulate_trajectory(target_interval, 0, -1, 0.05)
            T_a, _, reason_Ta = control.simulate_trajectory(target_interval, 1, -1, 0.05)
            
        extreme_trajectories = [T_d, T_a]
        extreme_trajectories_reasons = [reason_Td, reason_Ta]
        
        save_data.save_obj(extreme_trajectories, save_folder, "step_2_extreme_trajectories")
        save_data.save_obj(extreme_trajectories_reasons, save_folder, "step_2_extreme_trajectories_reasons")
    else:
        extreme_trajectories = save_data.load_obj(save_folder, "step_2_extreme_trajectories")
        extreme_trajectories_reasons = save_data.load_obj(save_folder, "step_2_extreme_trajectories_reasons")
        

    xa_intersect = extreme_trajectories[1][-1]
    xd_intersect = extreme_trajectories[0][-1]


    """
    work out the steps to run after this point
    """
    if work_out_steps_to_run:
        """
        The subsequent steps too run are only a function of why the extreme
        trajectories stopped
            - Step 3 should only run if T_a intersects the x1axis
            - Step 4 should only run if T_a doesnt hit the x_1 or x2 axis                
        """      
        T_d_stop_conditon = extreme_trajectories_reasons[0]
        T_a_stop_condition = extreme_trajectories_reasons[1]
        

        """
        Run step 3 in the case where the T_a curve intersects the x1 axis
        """
        if T_a_stop_condition == "x2<x2_lower_lim": 
            run_step_3 = True            
        else:
            run_step_3 = False    

        """
        Step 4 should be run if the following criteria are met
        """
          
        if T_d_stop_conditon != "x1<x1_lower_lim" and \
            T_d_stop_conditon != "x2<x2_lower_lim":
            
            run_step_4 = True            
        else:
            run_step_4 = False               
        
        
        save_data.save_obj(run_step_3, save_folder, "run_step_3")
        save_data.save_obj(run_step_4, save_folder, "run_step_4")           
        
        
    else:
        run_step_3 = save_data.load_obj(save_folder, "run_step_3") 
        run_step_4 = save_data.load_obj(save_folder, "run_step_4")         
 

    return extreme_trajectories, xa_intersect, xd_intersect, constraint_curve,  run_step_3, run_step_4


def form_tajectory(manipulator, target_state):
    
    control = path_dynamics_controller(manipulator)
    T_l, _, reason = control.simulate_trajectory(target_state, 0, -1, 0.01)

    return T_l, reason

def perform_u_scan(manipulator, x1_last):
    "function that scans the axis and shows the min input at L=1"
    
    control = path_dynamics_controller(manipulator)
    evaluated_list = control.scan_input_signs_on_x1_axis(0, x1_last, number_of_steps=1000, L=1)
    
    return evaluated_list   


def Step_3(manipulator, control,\
           xa_intersect, \
           run_full_simulation=True, \
           save_folder="ROS_data"):

    if run_full_simulation:        
        evaluated_inputs = perform_u_scan(manipulator, xa_intersect[0])
    
        ux = [x[0] for x in evaluated_inputs]
    
        min_input = min(ux)
        max_input = max(ux)        
        
        if min_input >= 0:
            print("it is all valid")
            x = list(np.linspace(0, float(xa_intersect[0]), 100))
            y = list(0*np.linspace(0, float(xa_intersect[0]),100))
            lower_boundary = combine_to_tuples(x,y)
            
        elif max_input < 0:
            print("all values are -ve so one trajectory needed")
            
            offset = 4
            point = (xa_intersect[0] , xa_intersect[1] + offset)
            
            lower_boundary, _, reason = control.simulate_trajectory(point, 1, -1, 0.1)
            #print(reason)
        else:
            print("the sign changes so more work needed here")
            lower_boundary = [(0.0)]
            
        save_data.save_obj(lower_boundary, save_folder, "step_3_lower_boundary")

    else:

        lower_boundary = save_data.load_obj(save_folder, "step_3_lower_boundary") 
        
    lower_boundary_trajectories = lower_boundary

    return lower_boundary_trajectories 


def Step_4(manipulator, control, xd_intersect, xa_intersect, \
           constraint_curve, situation, extreme_trajectories, \
           run_boundary_sim_setup=True,\
           run_boundary_sim=True,\
           save_folder="ROS_data"):
    """
    First loop along the constraint curve and calculate the tangent, 
    support vector and the direction a max deceleration curve will go
    Create a list that contains the state and inner product so that
    the regions can be defined
    """
    #control = path_dynamics_controller(manipulator)
    #new_constraint_list = form_constraints_list(situation)
    #control.set_constraint_list(new_constraint_list)
    
    if run_boundary_sim_setup:
        i = 0
        constraint_curve_counter = 0
        #constraint_curve.pop(0)
        for state in constraint_curve:
            x1 = state[0]
            x2 = state[1]    
            #print("state ", state, "cc ", constraint_curve[constraint_curve_counter],"count ", constraint_curve_counter)
            if x1 > xd_intersect[0]:
                break
            else:
                """
                work out the direction of the minimum deceleration curve at each 
                point along the constraint curve. 
                """
                A, D = control.calc_upper_lower_bound_values(state)

                if D <= A:
                    #print(D, A, state)
                    traj_dir = np.array([x2, D], dtype=np.float32)
                    #print("traj dir", traj_dir)
                    """
                    approxmimate the gradient of the data on the constraint curve 
                    using 2 data points (the current and next point)
                    """
                    #print("state", state , "next state", constraint_curve[constraint_curve_counter+1])
                    #print()
                    next_state = constraint_curve[constraint_curve_counter+1]
                    next_x1 = next_state[0]
                    next_x2 = next_state[1]
                    delta_x1 = next_x1 - x1
                    delta_x2 = next_x2 - x2
                    
                    #print("state, next state ", state, next_state)
                    #calculate the gradient
                    approx_gradient = np.array([delta_x1, delta_x2], dtype=np.float32)
                    #print("approx_gradient ", approx_gradient)
                    #rotate it by 90 degrees up the way
                    support_vector = np.array([-1*delta_x2, delta_x1], dtype=np.float32)
                    
                    inner_product = np.dot(traj_dir, support_vector)            
                    #print("inner product ", inner_product)
                    #print("boundary state ", x1, x2)
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
            constraint_curve_counter= constraint_curve_counter+1
            
        #print("list ", inner_product_list_with_state)

        """
        The output of the previous part is inner_product_list_with_state
        
        next
        
        unpack the information on the inner products and save all the points in
        which the region changes. 
        
        A positive number means f(x)>=0 which means the
        trajectory will go outside the constraint in the next time step 
        
        A negative number means f(x)<0 which means the trajectory will go inside
        the constraint in the next time step
        """
        #initialise variables
        #print("list ", relevant_constraint_curve_section)
        i=0
        sign_change = True
        prev_inner_product = 1
        #print(inner_product_list_with_state)
        for state_inner_product in inner_product_list_with_state:
            
            #abstract the state and inner product 
            state = state_inner_product[0]
            inner_product = state_inner_product[1]
                        
            #check if the current state has a sign change included 
            if (inner_product >= 0 and prev_inner_product  < 0) or \
               (inner_product < 0 and prev_inner_product  >= 0):
                sign_change=True
                #print("in here")
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
        inner_product = inner_product_list_with_state[-1][1]
        #print("inner product", inner_product)
        if inner_product >= 0:
            start_sign = "+ve"
        else:
            start_sign = "-ve"
        
        
        region_boundaries.append(xd_intersect)  
        save_data.save_obj(region_boundaries, save_folder, "step_4_region_boundaries")
        save_data.save_obj(start_sign, save_folder, "step_4_start_sign")
    
    else:
        region_boundaries = save_data.load_obj(save_folder, "step_4_region_boundaries")        
        start_sign = save_data.load_obj(save_folder, "step_4_start_sign")
    
    #print("region boundaries = ", region_boundaries)
    print("number of region boundaries", len(region_boundaries))
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
        
        #print("first boundary is: ", region_boundaries)
        #print("xa_intersect", xa_intersect)

        """
        if the xa_intersect has an x2>0 and x1> some small distance from x1_axis
        then we should move the boundary accordingly
        case where the acceleration curve intersects the constraint curve
        """
        if xa_intersect[0] > 0.005 and  xa_intersect[1] >= region_boundaries[0][1]:
            print("HERE")
            #if xa is on the constraint curve find the region boundary that needs moved
            i=0
            while region_boundaries[i][1] < xa_intersect[1] and\
                i < len(region_boundaries) - 1:
                    
                i = i + 1
            """
            the output i is the index of the region 1 further on than the one 
            that needs adjusted
            """            
            index_that_needs_moved = i - 1
            print(index_that_needs_moved)
            region_boundaries[index_that_needs_moved] = xa_intersect
        
        #print("after operation", region_boundaries[0])
        """
        If the start sign is -ve then we can add the entire first segment to the
        solution
        """
        #print(start_sign)

        if start_sign == "-ve":
            x1_low = region_boundaries[-(region_number+1)][0]
            x1_high = region_boundaries[-(region_number)][0]
            segment = extract_constraint_curve_segment(constraint_curve, x1_low, x1_high)

            #always append on the actual boundary to ensure proper connection
            segment.append(region_boundaries[-(region_number)])
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
        increment_const = 0.01
        slide_down_increment = increment_const
        
        sym_x1, sym_x2\
        = symbols('x1 x2')
        #print("len ", len(region_boundaries))
        while next_region_number <= len(region_boundaries): #and i <50:
            #set the boundary state as the last number
            boundary_state = region_boundaries[-region_number]
            next_boundary_state = region_boundaries[-(region_number + 1)]
            
            #slide down the line a small bit for the algorithm to work
            if i == 0:
                slid_down_slightly_state = (boundary_state[0], boundary_state[1] - slide_down_increment)
            else:
                if current_region_sign == "+ve":
                    slid_down_slightly_state = (boundary_state[0], prev_T_boundary_intersect[1] - slide_down_increment)                
                else:
                    slid_down_slightly_state = (boundary_state[0], prev_T_boundary_intersect[1])

            #print("current sign = ", current_region_sign)                    
            """
            adjust the constraints to be within an interval window so trajectories
            can correctly be formed
            """
            x1_lower_lim = next_boundary_state[0]
            x1_upper_lim = boundary_state[0]
            new_constraint_list = [-sym_x1 + x1_lower_lim,\
                                   sym_x1 - x1_upper_lim,\
                                   control.constraint_list[2],\
                                   control.constraint_list[3],\
                                   control.constraint_list[4]\
                                       ]
                
            control.set_constraint_list(new_constraint_list)
            #print("new constraint set ", new_constraint_list)
            
            """
            ensure a sufficent integration resolution is always used
            """
            space_between_boundaries = boundary_state[0] - next_boundary_state[0]
            if space_between_boundaries < 0.01:
                integration_step = space_between_boundaries/2
            else:
                integration_step = 0.01
            """
            integrate trajectory
            """            
            T, _, reason = control.simulate_trajectory(slid_down_slightly_state, L=0,\
                                                    direction=-1, T=integration_step)
            

            print("traj ", len(T), reason)
            print("boundary ", boundary_state, next_boundary_state)
            prev_T_boundary_intersect = T[-1]
            
            """
            #testing segment
            if slid_down_slightly_state[0] > 0.15 and slid_down_slightly_state[0] < 0.4:
                print("=====================")
                print("first state", slid_down_slightly_state, "last state", prev_T_boundary_intersect)
                print("boundaries", boundary_state, "next one", next_boundary_state)
                print("trajectory length", len(T))
                print("trajectory", T)
                print("reason", reason)
                print("region sign", current_region_sign)
                print("=====================")                
            """
            #print("reason", reason, boundary_state, next_boundary_state, space_between_boundaries)
            #print("space between boundaries ", space_between_boundaries)
            if current_region_sign == "+ve":                
                #place this append here for clearness
                #there is an identical one in the elif
                upper_boundary_trajectories.append(T)
                if reason == "L>U":
                    slide_down_increment = increment_const                                    
            #If the region sign is -ve we will either hit the constraint or reach the 
            #x1<x1_lower_lim_boundary

            elif current_region_sign=="-ve":
                
                """
                If the x1_lower_lim boundary is intersected things are easy
                
                else the constraint curve is intersected so we need to extract the relevant parts
                and add them to the trajectory
                """
                if reason == "x1<x1_lower_lim":
                    slide_down_increment = 0
                else:    
                    slide_down_increment = increment_const
                    """
                    find the index on the constraint curve that relates to the end
                    of the region
                    """
                    #print("in here")
                    #print("before ",prev_T_boundary_intersect)
                    j = 0
                    while constraint_curve[j][0] < next_boundary_state[0]:
                        j = j + 1
                    
                    constraint_curve_lower_bound_index = j
                    #print(constraint_curve[constraint_curve_lower_bound_index])
                    #print("intersection ->", prev_T_boundary_intersect)
                    
                    j = constraint_curve_lower_bound_index                
                    while constraint_curve[j][0] <= prev_T_boundary_intersect[0]:
                        j = j + 1
                    constraint_curve_upper_bound_index = j-1
                    #print("cc up", constraint_curve[constraint_curve_upper_bound_index])
                    
                    T_constraint =  constraint_curve[constraint_curve_lower_bound_index:constraint_curve_upper_bound_index]
                    #print("tcons", T_constraint)
                    T_constraint.reverse()
                    T.extend(T_constraint)
                    prev_T_boundary_intersect = T[-1]

                upper_boundary_trajectories.append(T)
                            
            region_number = region_number + 1
            next_region_number = region_number + 1
            i = i + 1

            #print("about to plot")
            #extreme_trajectories.append(T)
            #plotting.plot_multiple_trajectories(extreme_trajectories,\
            #                                    "f",\
            #                                    "paper_plots/",\
            #                                    save=False)
            #print("plotted")
        
            if current_region_sign == "+ve":
                current_region_sign= "-ve"
            elif current_region_sign=="-ve":
                current_region_sign = "+ve"
            
        save_data.save_obj(upper_boundary_trajectories, save_folder, "step_4_upper_boundary_trajectories")            
    else:
        upper_boundary_trajectories = save_data.load_obj(save_folder, "step_4_upper_boundary_trajectories")

    return upper_boundary_trajectories   

    
def form_constraints_list(situation,\
                          x1_lower_lim=0,\
                          x1_upper_lim=1,\
                          x2_lower_lim=0):
    """
    forms the constraints in the form each element should evaluate to zero
    with a state subed in if we are withing the constraints
    
    state: state to test [x1, x2]
    constraints: 
    [   -x1 + x1_lower_lim, 
        x1 - x1_upper_lim, 
        -x2 + x2_lower_lim,
        A(x)-D(x), 
        C(x)
    ]
    generally
    x1_lower_lim will be a float (usually 0)
    x1_upper_lim will be a float (usually 1)
    x2_lower_lim will be a float (usually 0)
    A(x)-D(x) will be an expression ()
    C(x) will be an expression for the other constraints
    """
    #print("situation", situation)
    x1, x2, Ax, Dx\
    = symbols('x1 x2 Ax Dx')
    constraints = [-x1 + x1_lower_lim]
    constraints.append(x1 - x1_upper_lim)
    constraints.append(-x2 + x2_lower_lim)
    constraints.append(Dx - Ax)
    Cx = generate_arbitrary_constraint_set_data(situation, return_sympy=True)
    constraints.append(Cx)
    
    
    return constraints

def generate_arbitrary_constraint_set_data(situation, return_sympy=False):
    
    warnings.filterwarnings("ignore")
    if return_sympy == True:
        x1, x2\
        = symbols('x1 x2')
        
    x = np.linspace(0, 1, 10000)  
    
    if situation == 1:
        y = x + 10
        #print("y is ", y)
        if return_sympy == True:
            expression = x2 - (x1 + 10)
            return expression 
        
    elif situation==2:
        y = 4*np.sin(10*x+5) - 2*np.sin(18*x*x*x) + 7
        
        if return_sympy == True:
            expression = x2 - (4*sympy.sin(10*x1+5) - 2*sympy.sin(18*x1*x1*x1) + 7)
            return expression 
    
    elif situation==3:
        y = 4*np.sin(10*x+5) - 2*np.sin(18*x*x*x) + 10
        #print("y is ", y)
        if return_sympy == True:
            expression = x2 - (4*sympy.sin(10*x1+5) - 2*sympy.sin(18*x1*x1*x1) + 7)
            return expression             
        
    if return_sympy == False:
        z=np.polyfit(x,y, 20) #x and y describe function to be approximated, the number is the order of polynomial
    
        y_calc = np.polyval(z, x)
        
        trajectory = combine_to_tuples(x, y_calc)
    
        return trajectory


def extract_constraint_curve_segment(constraint_curve, lower_x1_bound, upper_x1_bound):
    i=0
    segment = []

    """
    go through the points on the constraint curve until checking them against the lower 
    and upper bounds to extract the relevant portion
    """

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

def create_control_trajectory(current_time,\
                              control_type,\
                              control,\
                              start_state,\
                              extreme_trajectories, \
                              lower_boundary_trajectories, \
                              upper_bound_trajectories, \
                              simulate_controller,\
                              target_interval,\
                              save_folder="default_folder"):
      


    if simulate_controller == True:
        """
        format the entire lower bound values for the controller
        """
        lb_traj = list(lower_boundary_trajectories)
        #print(lb_traj[0])
        Ta = extreme_trajectories[1]
        Ta.reverse()
        lb_traj.extend(Ta)
        lower_bound_curve = lb_traj
        
        """
        format the upper bound trajectorues for the controller
        """
        ub_traj = [item for sublist in upper_bound_trajectories for item in sublist]
        
        Td = extreme_trajectories[0]
        Td.reverse()
        ub_traj.reverse()
        ub_traj.extend(Td)
        upper_bound_curve = list(ub_traj)
       
        if control_type == "1":         
            print("here")
            L = [control_type, lower_bound_curve, upper_bound_curve]
        else:
            L = [control_type]
        control_trajectory, actuation_levels,\
        u_values, u_val_with_state, L_val_with_state, A_list, D_list, \
        A_list_with_state, D_list_with_state = control.convex_controller_trajectory(start_state,\
                                                 lower_bound_curve,\
                                                 upper_bound_curve,\
                                                 epsilon = 0.01,\
                                                 control_strategy=L)
        
        torque_vector, tau_1_val_with_state, tau_2_val_with_state, torque_vector_with_state = control.compute_torques_required(u_val_with_state)
        
        
        save_data.save_obj(lower_bound_curve, save_folder, "lower_bound_curve")        
        save_data.save_obj(upper_bound_curve, save_folder, "upper_bound_curve")        
                
        save_data.save_obj(control_trajectory, save_folder, "8_1_control_trajectory_fc")
        save_data.save_obj(actuation_levels, save_folder, "8_1_actuation_level_fc")
        save_data.save_obj(u_values, save_folder, "8_1_u_fc")
        save_data.save_obj(u_val_with_state, save_folder, "8_1_u_val_state_fc")
        save_data.save_obj(L_val_with_state, save_folder, "8_1_L_val_state_fc")
        
        save_data.save_obj(A_list, save_folder, "8_1_A_list_fc")
        save_data.save_obj(D_list, save_folder, "8_1_D_list_fc")
        
        save_data.save_obj(A_list_with_state, save_folder, "8_1_A_list_with_state_fc")
        save_data.save_obj(D_list_with_state, save_folder, "8_1_D_list_with_state_fc")
        
        save_data.save_obj(torque_vector, save_folder, "8_1_torque_vector_fc")
        save_data.save_obj(tau_1_val_with_state, save_folder, "8_1_tau_1_val_with_state_fc")
        save_data.save_obj(tau_2_val_with_state, save_folder, "8_1_tau_2_val_with_state_fc")
        save_data.save_obj(torque_vector_with_state, save_folder, "8_1_torque_vector_with_state_fc")
        
    else:

        lower_bound_curve = save_data.load_obj(save_folder, "lower_bound_curve")  
        upper_bound_curve = save_data.load_obj(save_folder, "upper_bound_curve")                     
        
        control_trajectory = save_data.load_obj(save_folder, "8_1_control_trajectory_fc")  
        actuation_levels = save_data.load_obj(save_folder, "8_1_actuation_level_fc")             
        u_values = save_data.load_obj(save_folder, "8_1_u_fc")      
        u_val_with_state = save_data.load_obj(save_folder, "8_1_u_val_state_fc")  
        L_val_with_state = save_data.load_obj(save_folder, "8_1_L_val_state_fc")
        
        A_list = save_data.load_obj(save_folder, "8_1_A_list_fc")  
        D_list = save_data.load_obj(save_folder, "8_1_D_list_fc")  
        
        A_list_with_state = save_data.load_obj(save_folder, "8_1_A_list_with_state_fc")  
        D_list_with_state = save_data.load_obj(save_folder, "8_1_D_list_with_state_fc")  
    
        torque_vector  = save_data.load_obj(save_folder, "8_1_torque_vector_fc")  
        tau_1_val_with_state = save_data.load_obj(save_folder, "8_1_tau_1_val_with_state_fc")  
        tau_2_val_with_state = save_data.load_obj(save_folder, "8_1_tau_2_val_with_state_fc")  
        torque_vector_with_state = save_data.load_obj(save_folder, "8_1_torque_vector_with_state_fc")  
    

    if control_type == "1":
        #use control_type 1 as the example with the boundary of R as control guides
        bounds = [lower_bound_curve, upper_bound_curve]
        
        plotting.plot_control_type_1(control_trajectory[0],\
                                     bounds,\
                                     target_interval,\
                                     filepath="paper_plots/",\
                                     save=False)
        print(L_val_with_state[0])
        
        plotting.plot_L_v_x1(L_val_with_state, save=True,filepath=save_folder,\
                    filename="actuation_level_control_type_1", title="$\lambda$ vs $x_1$")
    
    elif control_type == "6":
        #use control_type 6 as the example with the boundary of R as control guides
        bounds = control.get_control_guides(control_type)      
        admissible_bounds = [lower_bound_curve, upper_bound_curve]
        
        plotting.plot_control_type_2(control_trajectory[0],\
                                     bounds,\
                                     admissible_bounds,\
                                     target_interval,\
                                     filepath="paper_plots/",\
                                     save=False)
        plotting.plot_L_v_x1(L_val_with_state, save=True,filepath="paper_plots/",\
                    filename="actuation_level_control_type_2", title="$\lambda$ vs $x_1$")
    


    elif control_type == "5":
        #use control_type 6 as the example with the boundary of R as control guides
        bounds = control.get_control_guides(control_type)      
        admissible_bounds = [lower_bound_curve, upper_bound_curve]
        
        plotting.plot_control_type_3(control_trajectory[0],\
                                     bounds,\
                                     admissible_bounds,\
                                     target_interval,\
                                     filepath="paper_plots/",\
                                     save=False)
        plotting.plot_L_v_x1(L_val_with_state, save=True,filepath="paper_plots/",\
                    filename="actuation_level_control_type_3", title="$\lambda$ vs $x_1$")
        
    
    
    elif control_type == "4":
        #use control_type 6 as the example with the boundary of R as control guides
        bounds = control.get_control_guides(control_type)      
        admissible_bounds = [lower_bound_curve, upper_bound_curve]
        
        plotting.plot_control_type_4(control_trajectory[0],\
                                     bounds,\
                                     admissible_bounds,\
                                     target_interval,\
                                     filepath="paper_plots/",\
                                     save=False)
        plotting.plot_L_v_x1(L_val_with_state, save=True,filepath="paper_plots/",\
            filename="actuation_level_control_type_3", title="$\lambda$ vs $x_1$")

    return control_trajectory, u_values


def create_joint_plots(path_def):
    
    
    path_type = path_def[0]
    print("the path type is:", path_type)
    #print(path_def)
    #if it is a straight line the problem is easy
    if path_type == 'joint_space_straight_line':
        
        q_0 = path_def[1][0]
        q_1 = path_def[2][0]
        
        #q0 = np.array(q_0)
        #q1 = np.array(q_1)
        
        q0m = np.linspace(q_0[0], q_1[0], num=50)
        q1m = np.linspace(q_0[1], q_1[1], num=50)
        q2m = np.linspace(q_0[2], q_1[2], num=50)
        q3m = np.linspace(q_0[3], q_1[3], num=50)
        q4m = np.linspace(q_0[4], q_1[4], num=50)
        q5m = np.linspace(q_0[5], q_1[5], num=50)
        
        
        
        N = np.linspace(0, len(q0m), num=len(q0m))
        
        #print(q0m)        
        #print(q1m)
        #print(q2m)
        #print(q3m)        
        #print(q4m)
        #print(q5m)
        plotting.plot_joints(q0m, q1m, q2m, q3m, q4m, q5m, N)
        
        #print("start", q_0)
        #print("end", q_1)
    