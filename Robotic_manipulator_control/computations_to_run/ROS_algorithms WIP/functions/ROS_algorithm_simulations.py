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

def generate_extreme_trajectories_to_target(manipulator, target_state, return_reason=False):
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
    evaluated_list = control.scan_input_signs_on_x1_axis(0, target_state, number_of_steps=10, L=1)
    
    return evaluated_list   


def Step_1(current_time,\
           robot,\
           simulation_parameters,\
           run_full_simulation=True,\
           save_folder="ROS_data/"):
    
    if run_full_simulation:    
        
        manipulator = two_dof_planar_robot(robot, current_time)   
        manipulator.run_full_path_dynamics_analysis(simulation_parameters['line_definition'], \
                                                    simulation_parameters['x1_lim'], \
                                                    simulation_parameters['x2_lim'])
        
        #simulate the kinematic movements of the robot
        manipulator.simulate_trajectory(manipulator.qs, 0.001)
            
        save_data.save_obj(manipulator, save_folder, "step_1_manipulator")        
    else:
        manipulator = save_data.load_obj(save_folder, "step_1_manipulator")  
    
    
    return manipulator
        
def Step_2(current_time, manipulator,\
           target_state, run_curve_simulations=True,\
           run_constraint_simulations=True,\
           run_ROS_boundary_xd_sim=True, run_ROS_boundary_xa_sim=True,\
           run_extract_boundaries_from_extreme_trajectories = True,\
           save_folder="ROS_data/", work_out_steps_to_run = True,\
           situation_select=0):


        
    if run_curve_simulations:       
        #produce an extreme trajectory set from the extreme trajectories
        control = path_dynamics_controller(manipulator)
        """
        check if the 
        """
        if isinstance(target_state,list):
            print("we have an interval target")
            T_d, _, reason_Td = control.simulate_trajectory(target_state[0], 0, -1, 0.05)
            T_a, _, reason_Ta = control.simulate_trajectory(target_state[1], 1, -1, 0.05)
        else:
            print("we have a single state target")
            T_d, _, reason_Td = control.simulate_trajectory(target_state, 0, -1, 0.05)
            T_a, _, reason_Ta = control.simulate_trajectory(target_state, 1, -1, 0.05)
            
        extreme_trajectories = [T_d, T_a]
        extreme_trajectories_reasons = [reason_Td, reason_Ta]
        
        save_data.save_obj(extreme_trajectories, save_folder, "step_2_extreme_trajectories")
        save_data.save_obj(extreme_trajectories_reasons, save_folder, "step_2_extreme_trajectories_reasons")
    else:
        extreme_trajectories = save_data.load_obj(save_folder, "step_2_extreme_trajectories")
        extreme_trajectories_reasons = save_data.load_obj(save_folder, "step_2_extreme_trajectories_reasons")
        
    if run_constraint_simulations:

        #generate_arbitrary_constraint_set()
        print("situation is ", situation_select)
        soft_constraint_curve = generate_arbitrary_constraint_set_data(situation_select)    
        #print(len(manipulator.boundary_points), len(soft_constraint_curve) )
        min_data = mm.min_function_two_constraints(manipulator.boundary_points, soft_constraint_curve)
        
        constraint_curve = [item for sublist in min_data for item in sublist]
        #constraint_curve = manipulator.boundary_points
        save_data.save_obj(constraint_curve, save_folder, "step_2_constraint_curve")        
        
    else:
        constraint_curve = save_data.load_obj(save_folder, "step_2_constraint_curve") 

    #define the xa intersect and xd intersect    
    if run_ROS_boundary_xd_sim:
        xd_intersect = mm.find_intersect_in_trajectories(constraint_curve,\
                                              extreme_trajectories[0], plot=False) 
        #print("yeo", xd_intersect)
        T_d_stop_conditon = extreme_trajectories_reasons[0]
        T_d = extreme_trajectories[0]
        """
        If no intersection is detected we can assume either:
            - we left the region in which the system is defined L>U
            - We intersected the x2_axis (x1 = 0)
            - We intersected the x1_axis (x2 = 0)
        Each of these cases should be explored accordingly and the point
        xd_intersect should be correctly positioned
        """
        
        if xd_intersect == False or len(xd_intersect) == 3:
            if T_d_stop_conditon == "L>U":
                """
                If the trajectory has run out of the valid region it has likely
                crossed a boundary.
                    - The constraint_curve_index will allow the closest point 
                      to be isolated on the constraint curve
                    - The while loop finds the closes point on the trajectory
                      on the inside of the constraint
                """
                _, constraint_curve_index   = mm.find_closest_points_in_data([T_d[-1]], constraint_curve)
                xd_constraint = constraint_curve[constraint_curve_index]
                i = 0  
                
                while T_d[i][0] > xd_constraint[0] and i < len(T_d):
                    i = i + 1
                last_index_Td = i
                
                #save the intersect
                xd_intersect = T_d[last_index_Td]
                
            elif T_d_stop_conditon == "x1<0" or T_d_stop_conditon == "x1<0" :
                """
                if either of these things happen take the last point on the trajectory
                """
                xd_intersect = T_d[-1]
            
            
        save_data.save_obj(xd_intersect, save_folder, "step_2_xd")        
    else:
        xd_intersect = save_data.load_obj(save_folder, "step_2_xd") 
        

    if run_ROS_boundary_xa_sim:
        xa_intersect = mm.find_intersect_in_trajectories(constraint_curve,\
                                              extreme_trajectories[1], plot=False)
        #print(extreme_trajectories[1])
        if xa_intersect == (False, False, False):             
            xa_intersect = extreme_trajectories[1][-1]
            
        save_data.save_obj(xa_intersect, save_folder, "step_2_xa")
    else:
        xa_intersect = save_data.load_obj(save_folder, "step_2_xa") 
           
    if run_extract_boundaries_from_extreme_trajectories:
        
        Td = extreme_trajectories[0]
        Ta = extreme_trajectories[1]        
        """
        loop through each list and stop when the intersection is found
        The try excepts deal with cases of no intersection by defaulting to
        returning the whole list
        """

        try:
            i = 0  
            while Td[i][0] > xd_intersect[0] and i < len(Td):
                i = i + 1
            last_index_Td = i 
        except:
            last_index_Td = len(Td) - 1

        try:
            i=0
            while Ta[i][0] > xa_intersect[0] and i < len(Ta):
                i = i + 1
            last_index_Ta = i
        except:
            last_index_Ta = len(Ta) - 1
        
        boundaries_from_extreme_trajectories = [Td[0:last_index_Td], Ta[0:last_index_Ta]]
        save_data.save_obj(boundaries_from_extreme_trajectories, save_folder, "boundaries_from_extreme_trajectories")
    else:
        boundaries_from_extreme_trajectories = save_data.load_obj(save_folder, "boundaries_from_extreme_trajectories") 


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
        if T_a_stop_condition == "x2<0": 
            run_step_3 = True            
        else:
            run_step_3 = False    

        """
        Step 4 should be run if the following criteria are met
        """
          
        if T_d_stop_conditon != "x1<0" and \
            T_d_stop_conditon != "x2<0":
            
            run_step_4 = True            
        else:
            run_step_4 = False               
        
        
        save_data.save_obj(run_step_3, save_folder, "run_step_3")
        save_data.save_obj(run_step_4, save_folder, "run_step_4")           
        
        
    else:
        run_step_3 = save_data.load_obj(save_folder, "run_step_3") 
        run_step_4 = save_data.load_obj(save_folder, "run_step_4")         
        

    return extreme_trajectories, boundaries_from_extreme_trajectories, xa_intersect, xd_intersect,constraint_curve,  run_step_3, run_step_4
        
def generate_arbitrary_constraint_set_data(situation, return_sympy=False):
    
    warnings.filterwarnings("ignore")
    if return_sympy == True:
        x1, x2\
        = symbols('x1 x2')

    
    x = np.linspace(0, 1, 100000)  
    
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
        save_data.save_obj(region_boundaries, "ROS data/", "step_4_region_boundaries")
        save_data.save_obj(start_sign, save_folder, "step_4_start_sign")
    
    else:
        region_boundaries = save_data.load_obj("ROS data/", "step_4_region_boundaries")        
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
        
        """
        if xa_intersect[0] > 0.005 and  xa_intersect[1] >= region_boundaries[0][1]:
            
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

        if start_sign == "-ve":
            x1_low = region_boundaries[-(region_number+1)][0]
            x1_high = region_boundaries[-(region_number)][0]
            segment = extract_constraint_curve_segment(constraint_curve, x1_low, x1_high)
            #print("erbjgvre", segment)

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
        slide_down_increment = 1
        while next_region_number <= len(region_boundaries): #and i <2:
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
                Third reason is that the last point on T is outside the region
                """
                constraints = form_constraints_list(situation)
                last_state=T[-1]
                print("the last state is ", last_state)
                violated = check_if_constraints_violated_analytically(last_state,\
                                                           constraints,\
                                                            manipulator)
                
                if reason == "L>U" or difference > 0.02 or violated==True:
                    
                    """
                    find the index on the constraint curve that relates to the end
                    of the region
                    """
                    j = 0
                    while constraint_curve[j][0] < next_boundary_state[0]:
                        j = j + 1
                    
                    constraint_curve_lower_bound_index = j
                    """
                    loop through the trajectory to ensure it does not pass over the
                    constraint curve
                    extract the relevant region
                    """
                    #print("constrain index", constraint_curve_lower_bound_index, constraint_curve_upper_bound_index)
                    #print("T last", T[last_index_to_take_on_T], "con 1st", constraint_curve[constraint_curve_lower_bound_index], "con last", constraint_curve[constraint_curve_upper_bound_index])
                    j=0
                    for state in T:
                        violated =\
                            check_if_constraints_violated_analytically(state,\
                                                           constraints,\
                                                            manipulator)                        
                        if violated:
                            break
                            
                        j= j+1
                    #print("Last indec to take ", j)
                    last_index_to_take_on_T = j
                    """
                    clip the trajectory formed
                    """
                    T = list(T[0:last_index_to_take_on_T])

                    """
                    step along the constraint curve and find the first value to the right
                    """
                    j = constraint_curve_lower_bound_index                
                    while constraint_curve[j][0] <= T[-1][0]:
                        j = j + 1
                    constraint_curve_upper_bound_index = j                
                    
                    """
                    finally form the T for this part
                    """
                    T_constraint =  constraint_curve[constraint_curve_lower_bound_index:constraint_curve_upper_bound_index]
                    T_constraint.reverse()
                    #print(T_constraint)
                    
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

        save_data.save_obj(upper_boundary_trajectories, save_folder, "step_4_upper_boundary_trajectories")            
    else:
        upper_boundary_trajectories = save_data.load_obj(save_folder, "step_4_upper_boundary_trajectories")

    return upper_boundary_trajectories    

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
    
    
