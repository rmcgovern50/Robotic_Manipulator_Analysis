# -*- coding: utf-8 -*-
from robot_models import two_dof_planar_robot
from functions.controller import path_dynamics_controller
import numpy as np
import my_math as mm
import save_data as save_data
from my_sorting import combine_to_tuples
import sympy
from sympy import symbols
import warnings
import matplotlib.pyplot as plt

from ml_robot_models import model



def generate_extreme_trajectories_to_target(manipulator, \
                                            target_state, \
                                            return_reason=False):
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
    T_l, _, reason = control.simulate_trajectory(target_state, 0, -1, 0.01)
    T_a, _, reason = control.simulate_trajectory(target_state, 1, -1, 0.01)

    return [T_l, T_a]


def perform_u_scan(manipulator, target_state):
    "function that scans the axis and shows the min input at L=1"
    control = path_dynamics_controller(manipulator)
    evaluated_list = control.scan_input_signs_on_x1_axis(0, target_state, number_of_steps=1000, L=1)
    
    return evaluated_list   

def matlab_model_Step_1(current_time,\
           robot,\
           simulation_parameters,\
           eng,\
           run_full_simulation=True,\
           save_folder="step1_data/"):

        
    manipulator = model(eng, robot, simulation_parameters, current_time)

    if run_full_simulation:
        manipulator.run_full_path_dynamics_analysis()
        stuff2pickle = manipulator.return_sim_results()
        save_data.save_obj(stuff2pickle, save_folder, "step_1_manipulator_matlab")
        
    else:
        pickledStuff = save_data.load_obj(save_folder, "step_1_manipulator_matlab")  
        manipulator.set_sim_results(pickledStuff)
        
        
    return manipulator

def Step_1(current_time,\
           robot,\
           simulation_parameters,\
           run_full_simulation=True,\
           save_folder="ROS_data/"):
    
    if run_full_simulation:    
        
        manipulator = two_dof_planar_robot(robot, current_time)   
        manipulator.run_full_path_dynamics_analysis(simulation_parameters['path_definition'], \
                                                    simulation_parameters['x1_lim'], \
                                                    simulation_parameters['x2_lim'])
        print("path dynamics simulated")
        #simulate the kinematic movements of the robot
        manipulator.simulate_trajectory(manipulator.qs, 0.01)
            
        save_data.save_obj(manipulator, save_folder, "step_1_manipulator")        
    else:
        manipulator = save_data.load_obj(save_folder, "step_1_manipulator")  
    
    
    return manipulator
        
def Step_2(current_time, manipulator, control,\
           target_state, min_of_upper_constraints, max_of_lower_constraints,\
           run_constraint_simulations=True,\
           run_curve_simulations=True,\
           run_ROS_boundary_xd_sim=True, run_ROS_boundary_xa_sim=True,\
           run_extract_boundaries_from_extreme_trajectories = True,\
           save_folder="ROS_data/", work_out_steps_to_run = True):

    resultant_upper_constraint = min_of_upper_constraints
    resultant_lower_constraint = max_of_lower_constraints
    
    if run_curve_simulations:       
        #produce an extreme trajectory set from the extreme trajectories

        # T_d, _, reason_Td = control.simulate_trajectory(target_state[0], 0, -1, 0.05)
        # T_a, _, reason_Ta = control.simulate_trajectory(target_state[1], 1, -1, 0.05)
        control.select_lambda_x(float(0))
        T_d, _, _, _, _, _, _, _, _, reason_Td = control.simulate_trajectory_with_control(target_state[0], -1, 0.05)
        control.select_lambda_x(float(1))
        T_a, _, _, _, _, _, _, _, _, reason_Ta = control.simulate_trajectory_with_control(target_state[1], -1, 0.05)        
        #print("Td gives", T_d)            
        #print("Ta gives", T_a)
        extreme_trajectories = [T_d, T_a]
        extreme_trajectories_reasons = [reason_Td, reason_Ta]
        #print(extreme_trajectories_reasons)
        save_data.save_obj(extreme_trajectories, save_folder, "step_2_extreme_trajectories")
        save_data.save_obj(extreme_trajectories_reasons, save_folder, "step_2_extreme_trajectories_reasons")
    else:
        extreme_trajectories = save_data.load_obj(save_folder, "step_2_extreme_trajectories")
        extreme_trajectories_reasons = save_data.load_obj(save_folder, "step_2_extreme_trajectories_reasons")
        
    #define the xa intersect and xd intersect    
    if run_ROS_boundary_xd_sim:

        T_d = extreme_trajectories[0]
        xd_intersect = T_d[-1]
                        
        save_data.save_obj(xd_intersect, save_folder, "step_2_xd")        
    else:
        xd_intersect = save_data.load_obj(save_folder, "step_2_xd") 
        
    if run_ROS_boundary_xa_sim:
    
        xa_intersect = extreme_trajectories[1][-1]
        save_data.save_obj(xa_intersect, save_folder, "step_2_xa")
    else:
        xa_intersect = save_data.load_obj(save_folder, "step_2_xa")
           
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
        
        if T_a_stop_condition == "x2<x2_lower_lim" or T_a_stop_condition == "additional constraint violated": 
            run_step_3 = True            
        else:
            run_step_3 = False    

        """
        Step 4 should be run if the following criteria are met
        """
          
        if T_d_stop_conditon != "x2<x1_lower_lim" and \
            T_d_stop_conditon != "x2<x2_lower_lim":
                
            run_step_4 = True            
        else:
            run_step_4 = False               
        
        
        save_data.save_obj(run_step_3, save_folder, "run_step_3")
        save_data.save_obj(run_step_4, save_folder, "run_step_4")           
        
        
    else:
        run_step_3 = save_data.load_obj(save_folder, "run_step_3") 
        run_step_4 = save_data.load_obj(save_folder, "run_step_4")         
        

    return extreme_trajectories, xa_intersect, xd_intersect,  run_step_3, run_step_4
        
def generate_arbitrary_constraint_set_data(situation, return_sympy=False):
    
    warnings.filterwarnings("ignore")
    if return_sympy == True:
        x1, x2\
        = symbols('x1 x2')
        
    x = np.linspace(0, 1, 10000)  
    
    #situation 1-4
    if situation == 1 or situation == 2 or situation == 3:
        y = (7*(x-0.5))**2 +9
        
        if return_sympy == True:
            expression = x2 - (7*(x1-0.5))**2 - 9
            return expression 
    elif situation==4:
        pass
    elif situation==5:
        pass
    elif situation==6:
        y = 4*np.sin(10*x) + 2*np.sin(18*x) + 15
        
        if return_sympy == True:
            expression = x2 - 4*sympy.sin(10*x1) - 2*sympy.sin(18*x1) - 15
            return expression
    elif situation==7:
        y = (8*(x-0.5))**2 +9
        
        if return_sympy == True:
            expression = x2 - (8*(x1-0.5))**2 - 9
            return expression        
    elif situation==8 or situation==8.1 or situation==8.2:
        y = 4*np.sin(10*x+5) - 2*np.sin(18*x*x*x) + 10
        
        if return_sympy == True:
            expression = x2 - (4*sympy.sin(10*x1+5) - 2*sympy.sin(18*x1*x1*x1) + 10)
            return expression 

    if return_sympy == False:
        z=np.polyfit(x,y, 20) #x and y describe function to be approximated, the number is the order of polynomial
    
        y_calc = np.polyval(z, x)
        
        trajectory = combine_to_tuples(x, y_calc)
    
        return trajectory
        
def Step_3(manipulator, xa_intersect, run_full_simulation=True, save_folder="ROS_data"):

    if run_full_simulation:  
        evaluated_inputs = perform_u_scan(manipulator, xa_intersect[0])
        ux = [x[0] for x in evaluated_inputs]
        
        min_input = min(ux)
        
        if min_input >= 0:
            print("it is all valid")
            x = list(np.linspace(0, float(xa_intersect[0]), 100))
            y = list(0*np.linspace(0, float(xa_intersect[0]),100))
            lower_boundary = combine_to_tuples(x,y)
            
        else:
            print("more work needed")
            lower_boundary = [(0,0)]
        save_data.save_obj(lower_boundary, save_folder, "step_3_lower_boundary")
    else:
        lower_boundary = save_data.load_obj(save_folder, "step_3_lower_boundary") 
    lower_boundary_trajectories = lower_boundary
    
    return lower_boundary_trajectories 

def Step_4(manipulator, xd_intersect, xa_intersect, \
           constraint_curve, situation, run_boundary_sim_setup=True,\
           run_boundary_sim=True,\
           save_folder="ROS_data"):
    """
    First loop along the constraint curve and calculate the tangent, 
    support vector and the direction a max deceleration curve will go
    Create a list that contains the state and inner product so that
    the regions can be defined
    """
    control = path_dynamics_controller(manipulator)
    new_constraint_list = form_constraints_list(situation)
    control.set_constraint_list(new_constraint_list)
    

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
                D, A = control.calc_upper_lower_bound_values(state)

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
        increment_const = 0.1
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
            if space_between_boundaries < 0.02:
                integration_step = space_between_boundaries/2
            else:
                integration_step = 0.02
            """
            integrate trajectory
            """            
            if(boundary_state[0] >0.34 and boundary_state[0] <0.35):
                print("what happens here?")
                
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

def find_sx_roots(poly_coeff_list, interval_list):

    """
    function to extract roots of sx which are the boundaries between regions
    """    

    new_interval_boundary_list = [(0,0)]
    boundary_type = ["el to be removed"]#will contain a string describing behaviours
    #determine interval boundaries
    for coeff, interval in zip(poly_coeff_list, interval_list):    
        #calculate roots of the polynomials
        r = np.roots(coeff)
        #extract real parts
        real_roots = r.real[abs(r.imag)<1e-5]
        #clip to only extract routes in the relevant interval
        real_roots = np.sort(real_roots[(real_roots >= interval[0]) & (real_roots <= interval[1])], axis=None)
        #print("interval and roots", interval, real_roots)
        #for each root check the derivative
        for root in real_roots:
            derivative_poly = np.polyder(coeff)
            d_evalu =np.polyval(derivative_poly, root) 
            new_interval_boundary_list.append((root))            
            if d_evalu < 0:
                boundary_type.append("positive to negative")
            elif d_evalu > 0:
                boundary_type.append("negative to positive")
            elif d_evalu == 0:
                boundary_type.append("max or min")
    
    new_interval_boundary_list.pop(0)
    boundary_type.pop(0) 
    
    return new_interval_boundary_list, boundary_type

def extend(manipulator, control, region_of_interest, poly_coeff_list,\
           interval_list, constraint_data, constraint_to_add, actuation_level,\
           integration_step = 0.001):

    control.select_lambda_x(float(actuation_level))
    sx_roots, _ = find_sx_roots(poly_coeff_list, interval_list)

    """
    form full interval_boundary list in the region of interest
    """
    x1_final = region_of_interest[0][0]
    x1_initial = region_of_interest[1][0]

    region_boundary_list = [x1_final]
    region_boundary_list.extend(sx_roots)
    region_boundary_list.append(x1_initial)

    region_boundary_list = np.array(region_boundary_list)
    region_boundary_list = np.sort(region_boundary_list[(region_boundary_list >= x1_final) & (region_boundary_list <= x1_initial)], axis=None)
    region_boundary_list = np.flip(region_boundary_list)

    print("the region boundaries are ", region_boundary_list)

    current_region_sign_list = ["unset"]
    i=0
    while i < len(region_boundary_list)-1:
        midpoint = (region_boundary_list[i] + region_boundary_list[i+1])/2
        
        sx_eval = evaluate_sx(midpoint, poly_coeff_list, interval_list)
        if sx_eval > 0:
            current_region_sign_list.append("+ve")
        if sx_eval <= 0:
            current_region_sign_list.append("-ve")   
        #print("the midpoint is: ", midpoint, sx_eval)
        i = i + 1

    current_region_sign_list.pop(0)

    #print("current region sign list", current_region_sign_list)
    extended_boundary = [(0,0)] #placeholder
    
    sym_x1, sym_x2\
    = symbols('x1 x2')
    
    i=0
    delta_mag = 0.1
    if actuation_level == 1:
        delta = delta_mag
    elif actuation_level == 0:
        delta = -delta_mag   
    
    current_state = region_of_interest[1]
    while i < len(current_region_sign_list):
        current_region_sign = current_region_sign_list[i]
        current_region_boundary = region_boundary_list[i]
        next_region_boundary = region_boundary_list[i+1]
        """
        adjust the constraints to be within an interval window so trajectories
        can correctly be formed
        """

        x1_lower_lim = next_region_boundary
        x1_upper_lim = current_region_boundary
        modified_constraints_list = [-sym_x1 + x1_lower_lim,\
                                sym_x1 - 1.0]#x1_upper_lim ]
        remaining_constraints = control.constraint_list[2:]
        new_constraint_list = modified_constraints_list + remaining_constraints
        #print(new_constraint_list)
        control.set_constraint_list(new_constraint_list)
        
        """
        A positive number means f(x)>=0 which means the
        trajectory will go outside the constraint in the next time step 
        
        A negative number means f(x)<0 which means the trajectory will go inside
        the constraint in the next time step
        """
        if current_region_sign == "-ve":
            #integrate backwards from the previous section to bridge the gap
            T, _, _, _, _, _, _, _, _, reason =\
                control.simulate_trajectory_with_control(current_state, direction=-1, T=integration_step)
            # T, _, reason = control.simulate_trajectory(current_state, L=actuation_level,\
            #               direction=-1, T=integration_step)
                
            extended_boundary.append(T) 
            current_state = T[-1]
            
            #if we do collide with the boundary add the whole thing as this whole region is -ve
            if reason == "L>U" or reason == "additional constraint violated" or reason == "x2<x2_lower_lim":
                print(i, " add this whole region from ", current_state[0], next_region_boundary)
                segment = extract_constraint_curve_segment(constraint_data, next_region_boundary, current_state[0])
                extended_boundary.append(segment)
                current_state = segment[0]
            
        elif current_region_sign == "+ve": 
            #we must step down and integrate to cross this region
            print(i, " integrate from ", current_region_boundary, next_region_boundary)  
            modified_state = (current_state[0], current_state[1] + delta)

            # T, _, reason = control.simulate_trajectory(modified_state, L=actuation_level,\
            #                           direction=-1, T=integration_step)
            T, _, _, _, _, _, _, _, _, reason =\
                 control.simulate_trajectory_with_control(modified_state, -1, T=integration_step)            
            
            print("The length of the trajectory is ", len(T), "the reason is ", reason, "final point is ", T[0])
            extended_boundary.append(T)
            current_state = T[-1]
            
            #if we collide with the constraint early make more boundaries repeat
            while reason == "L>U" or reason == "additional constraint violated" or reason == "x2<x2_lower_lim":
                modified_state = (current_state[0], current_state[1] + delta)
                # T, _, reason = control.simulate_trajectory(modified_state, L=0,\
                #                       direction=-1, T=integration_step)
                T, _, _, _, _, _, _, _, _, reason =\
                    control.simulate_trajectory_with_control(modified_state, -1, T=integration_step)
                extended_boundary.append(T)
                current_state = T[-1]
                extended_boundary.append(T)
                if reason == "x1<x1_lower_lim":
                    break
        i = i + 1
    
    extended_boundary.pop(0)
    extended_boundary = [item for sublist in extended_boundary for item in sublist]
    return extended_boundary
    
def evaluate_sx(x1_to_test, poly_coeff_list, interval_list):
    
    for coeff, interval in zip(poly_coeff_list, interval_list):
        if x1_to_test >= interval[0] and x1_to_test <= interval[1]:
            sx_val = np.polyval(coeff, x1_to_test)
            return sx_val

    
def min_of_constraints(natural_constraint_data, additional_constraint_expression):
    
    """
    The f(x1, x2) is formed by taking the equation of the additional constraint
    curve of the form
    
    x2 = g(x1), for convenience let us write it as x2 - g(x1) = 0
    
    the variable f allows us to evaluate
    
    x2 - g(x1)
    
    this allows us to infer that f > 0 relates to x2 > g(x1) which is above the constraint curve
    and f <= 0 relates to x2 <= g(x1) which is below the curve
    
    The loop below evaluates f at each point on the boundary where D(x) <= A(x)
    
    Since we want the min (A(x) = D(x), x2 = g(x1)) we do the following
    
    if f > 0 the natural boundary is above the additional constraint so we append
        x = (x1, -f(x1, 0)) to the list
    else f <=0
        x = (x1, x2)
        
        
    note that -f(x1, 0) = - (0 - g(x1)) = g(x1) = x2 on the additional constraint boundary
    
    """
    #print(additional_constraint_expression)
    if additional_constraint_expression != "N/A":
        x1, x2\
        = symbols('x1 x2')
        f = sympy.lambdify([x1, x2], additional_constraint_expression)
        
        x1_list = [x[0] for x in natural_constraint_data]
        x2_list = [x[1] for x in natural_constraint_data]
        
        new_constraint_states = [(0,0)]
        
        for x1_val, x2_val in zip(x1_list, x2_list):  
            f_evaluated = f(x1_val, x2_val)
            #print(f_evaluated)
            if f_evaluated > 0:
                new_constraint_states.append( (x1_val, -1*f(x1_val, 0))) 
            else:
                new_constraint_states.append((x1_val, x2_val)) 
    
        new_constraint_states.pop(0)
    else:
        print("No additional constraints for upper bound")
        new_constraint_states = natural_constraint_data
    
    return new_constraint_states

  
def max_of_constraints(natural_constraint_data, additional_constraint_expression):
    
    """
    The f(x1, x2) is formed by taking the equation of the additional constraint
    curve of the form
    
    x2 = g(x1), for convenience let us write it as x2 - g(x1) = 0
    
    the variable f allows us to evaluate
    
    x2 - g(x1)
    
    this allows us to infer that f > 0 relates to x2 > g(x1) which is above the constraint curve
    and f <= 0 relates to x2 <= g(x1) which is below the curve
    
    The loop below evaluates f at each point on the boundary where D(x) <= A(x)
    
    Since we want the min (A(x) = D(x), x2 = g(x1)) we do the following
    
    if f < 0 the natural boundary is below the additional constraint so we append
        x = (x1, -f(x1, 0)) to the list
    else f >=0
        x = (x1, x2)
        
        
    note that -f(x1, 0) = - (0 - g(x1)) = g(x1) = x2 on the additional constraint boundary
    
    """
    #print(additional_constraint_expression)
    if additional_constraint_expression != "N/A":
        x1, x2\
        = symbols('x1 x2')
        f = sympy.lambdify([x1, x2], additional_constraint_expression)
        
        x1_list = [x[0] for x in natural_constraint_data]
        x2_list = [x[1] for x in natural_constraint_data]
        
        new_constraint_states = [(0,0)]
        
        for x1_val, x2_val in zip(x1_list, x2_list):  
            f_evaluated = f(x1_val, x2_val)
            #print(f_evaluated)
            if f_evaluated < 0:
                new_constraint_states.append( (x1_val, -1*f(x1_val, 0))) 
            else:
                new_constraint_states.append((x1_val, x2_val)) 
    
        new_constraint_states.pop(0)
    else:
        print("No additional constraints for lower bound")
        new_constraint_states = natural_constraint_data
    
    return new_constraint_states




def evaluate_expression(expression, x1_start = 0, x1_end = 1, num_steps=100):
    
    x1, x2\
    = symbols('x1 x2')
        
    #sympy.solve(expression,)
    #f = sympy.lambdify([x1, x2], additional_constraint_expression)
    
    x1_list = np.linspace(x1_start, x1_end, num_steps)
    evaluated_states = [(0,0)]
    for x1_val in x1_list:
        exp = expression
        f = exp.subs(x1, x1_val)
        x2_val = float(sympy.solve(f, x2, numerical=True)[0])

        evaluated_states.append((x1_val, x2_val))

    evaluated_states.pop(0)
    return evaluated_states


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


def check_if_constraints_violated_analytically(state, constraints, manipulator):
    """
    Inputs
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
    x1, x2, Ax, Dx, Cx\
    = symbols('x1 x2 Ax Dx Cx')
    
    controller = path_dynamics_controller(manipulator)
    state_sub_list = [(x1, state[0]), (x2, state[1])]
    
    
    no_of_constraints_violated = 0
    i = 1
    for constraint in constraints:
        if i == 4:
            D, A = controller.calc_upper_lower_bound_values(state)
            limits_sub_list = [(Ax, A), (Dx, D)]
            
            constraint_value = constraint.subs(limits_sub_list) 
        else:
            constraint_value =  constraint.subs(state_sub_list) 
        
        if constraint_value > 0:
            no_of_constraints_violated = no_of_constraints_violated + 1
        
        #print("constraint value ", i, " ", constraint_value)
        i = i + 1
    
    if no_of_constraints_violated > 0:
        return True
    else:
        return False


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
    print("situation", situation)
    x1, x2, Ax, Dx\
    = symbols('x1 x2 Ax Dx')
    constraints = [-x1 + x1_lower_lim]
    constraints.append(x1 - x1_upper_lim)
    constraints.append(-x2 + x2_lower_lim)
    constraints.append(Dx - Ax)
    Cx = generate_arbitrary_constraint_set_data(situation, return_sympy=True)
    constraints.append(Cx)
    
    
    return constraints



def modify_constraints_list(upper_constraint_to_add="N/A",\
                          lower_constraint_to_add="N/A",\
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
        C_upper(x)
        C_lower(x)
        
    ]
    generally
    x1_lower_lim will be a float (usually 0)
    x1_upper_lim will be a float (usually 1)
    x2_lower_lim will be a float (usually 0)
    A(x)-D(x) will be an expression ()
    C_upper(x) will be an expression for the additional upper constraints
    C_lower(x) will be an expression for the additional lower constraints
    """

    x1, x2, Ax, Dx\
    = symbols('x1 x2 Ax Dx')
    constraints = [-x1 + x1_lower_lim]
    constraints.append(x1 - x1_upper_lim)
    constraints.append(-x2 + x2_lower_lim)
    constraints.append(Dx - Ax)
    
    if upper_constraint_to_add != "N/A":
        Cx_upper = upper_constraint_to_add
        constraints.append(Cx_upper)
        
    if lower_constraint_to_add != "N/A":
        Cx_lower = lower_constraint_to_add
        constraints.append(Cx_lower)
    
    return constraints

    
#===========Controller=====================================================  
  
def situation_1_controller(manipulator):
    
    control = path_dynamics_controller(manipulator)
        
    #define the origin to work out future bisections
    X0= (0.25,8)
    T1, _, _ = control.simulate_trajectory(X0,0.5, 1, 0.01, x1_lim=0.7)
       
    T2, _, _ = control.simulate_trajectory(X0, 0.75, 1, 0.01, x1_lim=0.45)
    
    T3, _, _ = control.simulate_trajectory(X0, 0.625, 1, 0.01, x1_lim=0.83)
    
    T4, _, _ = control.simulate_trajectory(X0, 0.6875, 1, 0.01, x1_lim=0.88)    

    T5, _, _ = control.simulate_trajectory(X0, 0.65625, 1, 0.01, x1_lim=0.88)    

    T6, _, _ = control.simulate_trajectory(X0, 0.671875, 1, 0.01, x1_lim=0.89)    
    """    
    X6 = (0.6890625,0)
    T7, _, _ = control.simulate_trajectory(X6, 1, 1, 0.05)    
    

    if control.find_intersect(T1, Ta) != False:
        print("Ta intersected with")
    elif control.find_intersect(T1, Td) != False:
        print("Td intersected with")
    else:
        print("neither intersected with")
    """
    
    return [T1, T2, T3, T4, T5, T6]#, T7]
    
    
    
def situation_8_1_controller(manipulator):
    
    control = path_dynamics_controller(manipulator)
    new_constraint_list = form_constraints_list(1)    
    control.set_constraint_list(new_constraint_list)
    
    #define the origin to work out future bisections
    X0= (0,4)
     
    T, _, _ = control.simulate_trajectory((0,4),0.95, 1, 0.1)#, x1_lim=1)
    Tlist = [T]
    
    #T, _, _ = control.simulate_trajectory((0,3),0.95, 1, 0.1)
    #Tlist = [T]
    #Tlist.append(T)
    """

    T, _, _ = control.simulate_trajectory((1,2.8), 0.51, -1, 0.005, x1_lim=1)
    #Tlist = [T]
    Tlist.append(T)
    """
    #T, _, _ = control.simulate_trajectory((0.31,4),0.52, 1, 0.05, x1_lim=1)
    #Tlist.append(T)


    #T6, _, _ = control.simulate_trajectory(X0, 0.390625, 1, 0.05, x1_lim=1)

    #T7, _, _ = control.simulate_trajectory(X0, 0.3828125, 1, 0.05, x1_lim=1)
    

    #T8, _, _ = control.simulate_trajectory(X0, 0.37890625, 1, 0.05, x1_lim=1)
    
    #T9, _, _ = control.simulate_trajectory(X0, 0.376953125, 1, 0.05, x1_lim=1)
           

    return  Tlist#[T1, T2]#, T3, T4, T5, T6, T7, T8, T9]    
    
    
def plot_constraints(natural_constraints,min_constraints, situation):
    """
    Takes in the natural bounds and the arifical ones then plots them for the paper
    """

    artifical_constraints = generate_arbitrary_constraint_set_data(situation)

    #print(artifical_constraint)
    #print(AD_bounds)

    x1n = [x[0] for x in natural_constraints]
    x2n = [x[1] for x in natural_constraints]

    x1a = [x[0] for x in artifical_constraints]
    x2a = [x[1] for x in artifical_constraints]
    
    
    x1m = [x[0] for x in min_constraints]
    x2m = [x[1] for x in min_constraints]
    
    x1lower_bound_curve = list(np.linspace(0,1,100))
    x2lower_bound_curve = list(0*np.linspace(0,1,100))
    #print(x1lower_bound_curve)
    
    x2left_wall = list(np.linspace(0,6.3,100))
    x1left_wall = list(np.linspace(0,0,100))    

    x2right_wall = list(np.linspace(0,10,100))
    x1right_wall = list(np.linspace(1,1,100))    
    
    
    fig = plt.figure()
    
    

    plt.plot(x1n, x2n, color='blue')              
    plt.plot(x1a, x2a, color='red')              
    plt.plot(x1m, x2m, color='green')              
    plt.plot(x1lower_bound_curve, x2lower_bound_curve, color='green')
    plt.plot(x1left_wall, x2left_wall, color='green')
    plt.plot(x1right_wall, x2right_wall, color='green')
        
    
    plt.xlabel("$x_1$")
    plt.ylabel("$x_2$")
    plt.xticks(np.arange(0, 1.1, step=0.1))
    plt.title("State space")

def plot_constraints_with_control(natural_constraints, min_constraints, control_trajectory, situation):
    """
    Takes in the natural bounds and the arifical ones then plots them for the paper
    """

    artifical_constraints = generate_arbitrary_constraint_set_data(situation)

    #print(artifical_constraint)
    #print(AD_bounds)

    x1n = [x[0] for x in natural_constraints]
    x2n = [x[1] for x in natural_constraints]

    x1a = [x[0] for x in artifical_constraints]
    x2a = [x[1] for x in artifical_constraints]
    
    x1m = [x[0] for x in min_constraints]
    x2m = [x[1] for x in min_constraints]

    x1c = [x[0] for x in control_trajectory]
    x2c = [x[1] for x in control_trajectory]
        
    control_trajectory
    
    x1lower_bound_curve = list(np.linspace(0,1,100))
    x2lower_bound_curve = list(0*np.linspace(0,1,100))
    #print(x1lower_bound_curve)
    
    x2left_wall = list(np.linspace(0,6.3,100))
    x1left_wall = list(np.linspace(0,0,100))    

    x2right_wall = list(np.linspace(0,10,100))
    x1right_wall = list(np.linspace(1,1,100))    
    
    
    fig = plt.figure()
    

    plt.plot(x1lower_bound_curve, x2lower_bound_curve, color='green', linewidth=4)
    plt.plot(x1left_wall, x2left_wall, color='green', linewidth=4)
    plt.plot(x1right_wall, x2right_wall, color='green', linewidth=4)
    plt.plot(x1m, x2m, color='green', linewidth=4)
    plt.plot(x1right_wall, x2right_wall, color='red', linewidth=4)
    plt.plot(x1right_wall, x2right_wall, color='red', linewidth=4)
        
    plt.plot(x1n, x2n, '--', color='blue', linewidth=2)              
    plt.plot(x1a, x2a, '--', color='orange', linewidth=2)  

    plt.plot(x1c, x2c, color='purple')              
    

    plt.xlabel("$x_1$")
    plt.ylabel("$x_2$")
    plt.xticks(np.arange(0, 1.1, step=0.1))
    plt.title("State space")

    
