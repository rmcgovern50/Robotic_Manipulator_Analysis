# -*- coding: utf-8 -*-
"""
Created on Thu Jan 15 2021

@author: Ryan McGovern

This script will use my model of a robot to calculate the dynamics
on a path and plot the admissible and inadmissable regions
"""

import sys
sys.path.append('../../../My_modules/my_basic_modules')
sys.path.append('../../../My_modules/my_control_modules')
sys.path.append('../../../Robotic_manipulator_control')
import functions.ROS_simulations_plotting as plotting
import functions.ROS_algorithm_simulations_updated as sim
#import functions.ml_ROS_algorithm_simulations as sim

import datetime as dt
import numpy as np
from my_sorting import combine_to_tuples
from functions.controller import path_dynamics_controller

import save_data as save_data
   


def save_data_to_csv(data):
    import csv   
    #data = [(1,2),(4,5),(7,8)]
    item_length = len(data[0])
    
    with open('constraint_data_example.csv', 'w') as test_file:
      file_writer = csv.writer(test_file)
      for i in range(item_length):
          file_writer.writerow([x[i] for x in data])

def find_ROS(situation, start_time):
    """
    This function will systematically run all simulations required to find the
    ROS
    #Step 1 - create a manipulator object    
    #Step 2 - form the extreme trajectories to the target
    #Step 3 - Form any extensions on the lower bound targets
    #Step 4 - Form any extensions on the upper bound targets
    
    The objective will be to perform the steps and plot the result after each
    """
    current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots  

    if situation==1:
        """
        situation 1 is the case where:
            - niether extreme trajectories actually intersect the x_2 axis
            - The x1 axis is a boundary between 0 and xa_intersect
            - The td extreme trajectory intersects the constraint curve
        
        This is a good example to show the top boundary algorithm working
        """
        
        folder_name = "example_situation_1/"
        target_state = (0.9, 8.5)
        
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
           
        print("Situation 1 parameters input")
        
    elif situation==2:
        """
        situation 2 is the case where:
            - the upper extreme trajectory intersect the constraint_curve
            - the lower one intersects the x2_axis
            - the x1 axis is negative
            
        This is a good example for a more obsure case
        
        """       
        
        
        folder_name = "example_situation_2/"
        target_state = (1, 1)
        
        #step 1
        robot = {'joint_masses': [0.25, 10],\
                 'link_lengths': [0.2,0.2],\
                 'actuator_limits':  [(-10,10), (-10,10)]\
                }
        simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                                  'x2_lim':  [0,20, 0.01],\
                                  'line_definition': ["straight line", [(0.34,-0.20),(0.34, 0.20)]],\
                                  'initial_state': (0,0),\
                                  'final_state': (1,11.5)\
                                  }        
        print("Situation 2 parameters input")


    elif situation==3:
        """
        situation 2 is the case where:
            -both extreme trajectories intersect the constraint_curve
            -the x1 axis is negative
            
            
        This is a good example for a more obsure case
        
        """
        
        folder_name = "example_situation_3/"
        target_state = (1, 5)
        
        #step 1
        robot = {'joint_masses': [0.25, 10],\
                 'link_lengths': [0.2,0.2],\
                 'actuator_limits':  [(-10,10), (-10,10)]\
                }
        simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                                  'x2_lim':  [0,20, 0.01],\
                                  'line_definition': ["straight line", [(0.34,-0.20),(0.34, 0.20)]],\
                                  'initial_state': (0,0),\
                                  'final_state': (1,11.5)\
                                  }        
        print("Situation 3 parameters input")

    elif situation==4:
        """
        situation 2 is the case where:
            -both extreme trajectories intersect the constraint_curve
            -the x1 axis is negative
            
            
        This is a good example for a more obsure case        
        """
        
        folder_name = "example_situation_4/"
        target_state = (1, 7.45)
        
        #step 1
        robot = {'joint_masses': [0.25, 10],\
                 'link_lengths': [0.2,0.2],\
                 'actuator_limits':  [(-10,10), (-10,10)]\
                }
        simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                                  'x2_lim':  [0,20, 0.01],\
                                  'line_definition': ["straight line", [(0.34,-0.20),(0.34, 0.20)]],\
                                  'initial_state': (0,0),\
                                  'final_state': (1,11.5)\
                                  }        
        print("Situation 4 parameters input")

    elif situation==5:
        """
        situation 1 is the case where:
            - niether extreme trajectories actually intersect the x_2 axis
            - The x1 axis is a boundary between 0 and xa_intersect
            - The td extreme trajectory intersects the constraint curve
        
        This is a good example to show the top boundary algorithm working
        """

        folder_name = "example_situation_5/"
        target_state = (0.9, 8.5)

        robot = {'joint_masses': [0.25, 0.25],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits': [(-10,10), (-10,10)]\
            }
        simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                              'x2_lim':  [0,20, 0.01],\
                              'line_definition': ["circular arc", [(0.1,0.30), 0.05]],\
                              'initial_state': (0,0),\
                              'final_state': (1,11.5)\
                              }
        print("Situation 5 parameters input")

    elif situation==6:
        """
        situation 6 is the case where:

        This is a good example to show the top boundary algorithm working
        """

        folder_name = "example_situation_6/"
        target_state = (0.9, 8.5)

        robot = {'joint_masses': [0.25, 0.25],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits': [(-10,10), (-10,10)]\
            }
        simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                              'x2_lim':  [0,20, 0.01],\
                              'line_definition': ["circular arc", [(0.1, 0.30), 0.05]],\
                              'initial_state': (0,0),\
                              'final_state': (1,11.5)\
                              }
        print("Situation 6 parameters input")

    elif situation==7:
        """
        situation 7 is the case where:

        This is a good example to show the top boundary algorithm working
        """

        folder_name = "example_situation_7/"
        target_state = (0.9, 8.5)

        robot = {'joint_masses': [0.25, 0.25],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits': [(-10,10), (-10,10)]\
            }
        simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                              'x2_lim':  [0,20, 0.01],\
                              'line_definition': ["circular arc", [(0.1, 0.30), 0.15]],\
                              'initial_state': (0,0),\
                              'final_state': (1,11.5)\
                              }
        print("Situation 7 parameters input")


    elif situation==8:
        """
        situation 8 is the case where:

        A complex top boundary for the paper
        """

        folder_name = "example_situation_8/"

        target_state = (1, 8.5)

        robot = {'joint_masses': [0.25, 0.25],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits': [(-10,10), (-10,10)]\
            }
        simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                              'x2_lim':  [0,20, 0.01],\
                              'line_definition': ["circular arc", [(0.1, 0.30), 0.15]],\
                              'initial_state': (0,0),\
                              'final_state': (1,11.5)\
                               }
        print("Situation 8 parameters input")

    elif situation==8.1:
        """
        situation 8 is the case where:

        A complex top boundary for the paper
        """

        folder_name = "example_situation_8.1/"

        #target_state = (1, 8.5)
        "always go [upper lower]"
        target_state = [(1, 8.5), (1, 2.8)]

        robot = {'joint_masses': [0.25, 0.25],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits': [(-10,10), (-10,10)]\
            }
        simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                              'x2_lim':  [0,20, 0.01],\
                              'line_definition': ["circular arc", [(0.1, 0.30), 0.15]],\
                              'initial_state': (0,0),\
                              'final_state': (1,11.5)\
                               }
        print("Situation 8.1 parameters input")

    elif situation==8.2:
        """
        situation 8 is the case where:

        A complex top boundary for the paper
        """

        folder_name = "example_situation_8.2/"

        #target_state = (1, 8.5)
        "always go [upper lower]"
        target_state = [(1, 8.5), (1, 2.8)]

        robot = {'joint_masses': [0.25, 0.25],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits': [(-10,10), (-10,10)]\
            }
        simulation_parameters = { 'x1_lim':[0, 1, 0.01],\
                              'x2_lim':  [0,20, 0.01],\
                              'line_definition': ["circular arc", [(0.1, 0.30), 0.15]],\
                              'initial_state': (0,0),\
                              'final_state': (1,11.5)\
                               }
        print("Situation 8.2 parameters input")

    #step 1
    manipulator = sim.Step_1(current_time,\
                             robot,\
                             simulation_parameters,\
                             run_full_simulation=False,\
                             save_folder=folder_name)
        
    print("Step 1 complete")
    step_1_run_time =  dt.datetime.now()             
    diff = step_1_run_time-start_time
    minutes = diff.seconds/60
    print("Step 1 took ", minutes, "minutes")
    #step 2
    extreme_trajectories,\
    xa_intersect,\
    xd_intersect,\
    constraint_curve,\
    run_step_3,\
    run_step_4 = \
        sim.Step_2(current_time,\
                       manipulator,\
                       target_state,\
                       situation,\
                       run_constraint_simulations=False,\
                       run_curve_simulations=False,\
                       run_ROS_boundary_xd_sim=False,\
                       run_ROS_boundary_xa_sim=False,\
                       save_folder=folder_name,\
                       work_out_steps_to_run=False,\
                       situation_select=situation)
    
    print("Step 2 complete")
    save_data_to_csv(constraint_curve)
    step_2_run_time=  dt.datetime.now()             
    diff = step_2_run_time-step_1_run_time
    minutes = diff.seconds/60
    print("Step 2 took ", minutes, "minutes")
    
    #print("intersecting states", xa_intersect, xd_intersect)
    print("Step 3 should be run? ", run_step_3)
    print("Step 4 should be run? ", run_step_4)

    #step 3
    #running is subject to the results from step 2
    if run_step_3:
        lower_boundary_trajectories = sim.Step_3(manipulator,\
                                                 xa_intersect,\
                                                 run_full_simulation=False,\
                                                 save_folder=folder_name)   
        print("Step 3 complete")
        step_3_run_time=  dt.datetime.now()             
        diff = step_3_run_time-step_2_run_time
        minutes = diff.seconds/60
        print("Step 3 took ", minutes, "minutes")
    #step 4
    #running is subject to step 2 output
    if run_step_4:
        upper_bound_trajectories = sim.Step_4(manipulator, \
                                              xd_intersect,\
                                              xa_intersect, \
                                              constraint_curve,\
                                              situation,\
                                              run_boundary_sim_setup=False,\
                                              run_boundary_sim=False,\
                                              save_folder=folder_name)
        print("step 4 complete")
        step_4_run_time=  dt.datetime.now()             
        diff = step_4_run_time-step_3_run_time
        minutes = diff.seconds/60
        print("Step 4 took ", minutes, "minutes")
    
    #final plotting functions
    trajectory_list = extreme_trajectories
    trajectory_list.append(constraint_curve)
    if run_step_3: trajectory_list.append(lower_boundary_trajectories)
    if run_step_4: trajectory_list.extend(upper_bound_trajectories)
    
    ###=======plotting==========
    #plotting.plot_multiple_trajectories(trajectory_list, current_time,"paper_plots/", save=False)    
    plotting.plot_multiple_trajectories([constraint_curve], current_time,"paper_plots/", save=False)
    manipulator.plot_workspace_movement(save=False, plot_start_end_points=True)    
    
    #soft_constraint = sim.generate_arbitrary_constraint_set_data(situation)    
    #plotting.plot_constraints(manipulator.boundary_points,\
                              #soft_constraint,\
                              #current_time,\
                              #save_file=False)
        
    manipulator.plot_joint_space(ms=1, save_file=False)    
    
    sim.plot_constraints(manipulator.boundary_points,constraint_curve, situation)

    if situation==1:
        pass
        # = sim.situation_1_controller(manipulator)
        #trajectory_list.pop(0)
        #trajectory_list.pop(0)
        #trajectory_list.pop(0)
        #trajectory_list.extend(controller_list)
        #plotting.plot_controller_sit_1(trajectory_list, current_time, save=False)
        
    elif situation==8.1:
        control = path_dynamics_controller(manipulator)
        new_constraint_list = sim.form_constraints_list(situation)
        control.set_constraint_list(new_constraint_list)
        
        #controller_to_use = "piecewise continuous"
        controller_to_use = "piecewise continuous"
        simulate_controller = False
        
        if controller_to_use == "piecewise continuous":
            if simulate_controller == True:
                start_state = (0.0,2 )        
                """
                format the entire lower bound values for the controller
                """
                lb_traj = list(lower_boundary_trajectories)
                #print(lb_traj[0])
                Ta = extreme_trajectories[1]
                Ta.reverse()
                lb_traj.extend(Ta)
                lower_bound_curve = lb_traj
                #print(lower_bound_curve)
        
                ub_traj = [item for sublist in upper_bound_trajectories for item in sublist]
                
                Td = extreme_trajectories[0]
                Td.reverse()
                ub_traj.reverse()
                
        
        
                ub_traj.extend(Td)
                #upper_bound_curve = [item for sublist in ub_traj for item in sublist]
                upper_bound_curve = list(ub_traj)
                
                control_trajectory, actuation_levels,\
                u_values, u_val_with_state, L_val_with_state, A_list, D_list, \
                A_list_with_state, D_list_with_state = \
                    control.convex_controller_trajectory(start_state,\
                                                         lower_bound_curve,\
                                                         upper_bound_curve,\
                                                         epsilon= 0.01 )

                torque_vector, tau_1_val_with_state, tau_2_val_with_state, torque_vector_with_state = control.compute_torques_required(u_val_with_state)


                save_data.save_obj(control_trajectory, folder_name, "8_1_control_trajectory_pwc")
                save_data.save_obj(actuation_levels, folder_name, "8_1_actuation_level_pwc")
                save_data.save_obj(u_values, folder_name, "8_1_u_fc")
                save_data.save_obj(u_val_with_state, folder_name, "8_1_u_val_state_pwc")
                save_data.save_obj(L_val_with_state, folder_name, "8_1_L_val_state_pwc")
                
                save_data.save_obj(A_list, folder_name, "8_1_A_list_pwc")
                save_data.save_obj(D_list, folder_name, "8_1_D_list_pwc")
                
                save_data.save_obj(A_list_with_state, folder_name, "8_1_A_list_with_state_pwc")
                save_data.save_obj(D_list_with_state, folder_name, "8_1_D_list_with_state_pwc")
                
                
                save_data.save_obj(torque_vector, folder_name, "8_1_torque_vector_fc")
                save_data.save_obj(tau_1_val_with_state, folder_name, "8_1_tau_1_val_with_state_pwc")
                save_data.save_obj(tau_2_val_with_state, folder_name, "8_1_tau_2_val_with_state_pwc")
                save_data.save_obj(torque_vector_with_state, folder_name, "8_1_torque_vector_with_state_pwc")
            else:
                control_trajectory = save_data.load_obj(folder_name, "8_1_control_trajectory_pwc")  
                actuation_levels = save_data.load_obj(folder_name, "8_1_actuation_level_pwc")             
                u_values = save_data.load_obj(folder_name, "8_1_u_fc")      
                u_val_with_state = save_data.load_obj(folder_name, "8_1_u_val_state_pwc")  
                L_val_with_state = save_data.load_obj(folder_name, "8_1_L_val_state_pwc")
                
                A_list = save_data.load_obj(folder_name, "8_1_A_list_fc")  
                D_list = save_data.load_obj(folder_name, "8_1_D_list_fc")  
                
                A_list_with_state = save_data.load_obj(folder_name, "8_1_A_list_with_state_pwc")  
                D_list_with_state = save_data.load_obj(folder_name, "8_1_D_list_with_state_pwc")  

                torque_vector  = save_data.load_obj(folder_name, "8_1_torque_vector_fc")  
                tau_1_val_with_state = save_data.load_obj(folder_name, "8_1_tau_1_val_with_state_pwc")  
                tau_2_val_with_state = save_data.load_obj(folder_name, "8_1_tau_2_val_with_state_pwc")  
                torque_vector_with_state = save_data.load_obj(folder_name, "8_1_torque_vector_with_state_pwc")               



            plotting.plot_both_tau_vs_x_sit_8_1(torque_vector_with_state, current_time, save=False)
        elif controller_to_use == "fully continuous":
            print("run the special controller")
            cs=5
            if simulate_controller == True:
                
                start_state = (0.0,2)        
                """
                format the entire lower bound values for the controller
                """
                lb_traj = list(lower_boundary_trajectories)
                #print(lb_traj[0])
                Ta = extreme_trajectories[1]
                Ta.reverse()
                lb_traj.extend(Ta)
                lower_bound_curve = lb_traj
                #print(lower_bound_curve)
                
                """
                format the upper bound trajectorues for the controller
                """
                ub_traj = [item for sublist in upper_bound_trajectories for item in sublist]
                
                Td = extreme_trajectories[0]
                Td.reverse()
                ub_traj.reverse()
                ub_traj.extend(Td)
                upper_bound_curve = list(ub_traj)
                
                
                control_trajectory, actuation_levels,\
                u_values, u_val_with_state, L_val_with_state, A_list, D_list, \
                A_list_with_state, D_list_with_state = control.convex_controller_trajectory(start_state,\
                                                         lower_bound_curve,\
                                                         upper_bound_curve,\
                                                         epsilon = 0.01,\
                                                         control_strategy=cs)
                
                torque_vector, tau_1_val_with_state, tau_2_val_with_state, torque_vector_with_state = control.compute_torques_required(u_val_with_state)

            
                save_data.save_obj(control_trajectory, folder_name, "8_1_control_trajectory_fc")
                save_data.save_obj(actuation_levels, folder_name, "8_1_actuation_level_fc")
                save_data.save_obj(u_values, folder_name, "8_1_u_fc")
                save_data.save_obj(u_val_with_state, folder_name, "8_1_u_val_state_fc")
                save_data.save_obj(L_val_with_state, folder_name, "8_1_L_val_state_fc")
                
                save_data.save_obj(A_list, folder_name, "8_1_A_list_fc")
                save_data.save_obj(D_list, folder_name, "8_1_D_list_fc")
                
                save_data.save_obj(A_list_with_state, folder_name, "8_1_A_list_with_state_fc")
                save_data.save_obj(D_list_with_state, folder_name, "8_1_D_list_with_state_fc")
                
                
                save_data.save_obj(torque_vector, folder_name, "8_1_torque_vector_fc")
                save_data.save_obj(tau_1_val_with_state, folder_name, "8_1_tau_1_val_with_state_fc")
                save_data.save_obj(tau_2_val_with_state, folder_name, "8_1_tau_2_val_with_state_fc")
                save_data.save_obj(torque_vector_with_state, folder_name, "8_1_torque_vector_with_state_fc")
                
                
            else:
                control_trajectory = save_data.load_obj(folder_name, "8_1_control_trajectory_fc")  
                actuation_levels = save_data.load_obj(folder_name, "8_1_actuation_level_fc")             
                u_values = save_data.load_obj(folder_name, "8_1_u_fc")      
                u_val_with_state = save_data.load_obj(folder_name, "8_1_u_val_state_fc")  
                L_val_with_state = save_data.load_obj(folder_name, "8_1_L_val_state_fc")
                
                A_list = save_data.load_obj(folder_name, "8_1_A_list_fc")  
                D_list = save_data.load_obj(folder_name, "8_1_D_list_fc")  
                
                A_list_with_state = save_data.load_obj(folder_name, "8_1_A_list_with_state_fc")  
                D_list_with_state = save_data.load_obj(folder_name, "8_1_D_list_with_state_fc")  

                torque_vector  = save_data.load_obj(folder_name, "8_1_torque_vector_fc")  
                tau_1_val_with_state = save_data.load_obj(folder_name, "8_1_tau_1_val_with_state_fc")  
                tau_2_val_with_state = save_data.load_obj(folder_name, "8_1_tau_2_val_with_state_fc")  
                torque_vector_with_state = save_data.load_obj(folder_name, "8_1_torque_vector_with_state_fc")  


            extra_analysis_plot_things = False

            if extra_analysis_plot_things==True:
                if cs == 2:
                    for x1 in np.linspace(0,1,20):
                        """
                        define the upper line
                        """
                        m1 = 2
                        x2_upper = m1*x1 + 3
                        """
                        define the lower line
                        """
                        m2 = 3
                        x2_lower = m2*x1 + 1
                        if x1==0:
                            lower_guide = [x2_lower]
                            upper_guide = [x2_upper]
                        else:    
                            lower_guide.append(x2_lower)
                            upper_guide.append(x2_upper)
                        
                if cs==3:                                  
                    for x1 in np.linspace(0,1,20):
                        x2_upper = 3*x1**2 + 3
                        """
                        define the lower curve
                        """
                        x2_lower = 3.5*x1**2 + 0.5
                        if x1==0:
                            lower_guide = [x2_lower]
                            upper_guide = [x2_upper]
                        else:    
                            lower_guide.append(x2_lower)
                            upper_guide.append(x2_upper) 
                            
                if cs==4:                                  
                    for x1 in np.linspace(0,1,20):
                        
                        x2_lower = 3*x1**2 + 3
                        x2_upper = 3*x1**2 + 3
                        if x1==0:
                            lower_guide = [x2_lower]
                            upper_guide = [x2_upper]
                        else:    
                            lower_guide.append(x2_lower)
                            upper_guide.append(x2_upper) 
                            
                if cs==5:                                  
                    for x1 in np.linspace(0,1,20):
                        
                        x2_lower = 3*x1 + 0.5
                        x2_upper = 3*x1 + 0.5
                        if x1==0:
                            lower_guide = [x2_lower]
                            upper_guide = [x2_upper]
                        else:    
                            lower_guide.append(x2_lower)
                            upper_guide.append(x2_upper) 
                            
                            
                #print(upper_guide)
                #print(lower_guide)
                #remove extremes
                trajectory_list.pop(2)
                boundary_list_len = len(trajectory_list)
                #add control trajectory
                trajectory_list.extend(control_trajectory)
                plotting.plot_controller_sit_8_1_with_guides(trajectory_list, upper_guide, \
                                                             lower_guide, current_time, \
                                                                 boundary_list_len, save=False)
                #A value plot    
                #plotting.plot_Ax_sit_8_1(A_list, current_time, save=False)
                #plotting.plot_Dx_sit_8_1(D_list, current_time, save=False)
                
                #plotting.plot_upper_constraint_vs_x_sit_8_1(A_list_with_state, current_time, save=False)
                #plotting.plot_lower_constraint_vs_x_sit_8_1(D_list_with_state, current_time, save=False)  
                
                #plotting.plot_tau_1_vs_x_sit_8_1(tau_1_val_with_state, current_time, save=False)
                #plotting.plot_tau_2_vs_x_sit_8_1(tau_2_val_with_state, current_time, save=False)                
                #plotting.plot_both_tau_vs_x_sit_8_1(torque_vector_with_state, current_time, save=False)
                
                
        print("control algorithm run")
        control_run_time=  dt.datetime.now()             
        diff = control_run_time-step_4_run_time
        minutes = diff.seconds/60
        print("control algorithm took", minutes, "minutes")
                
        #remove extremes
        trajectory_list.pop(2)
        boundary_list_len = len(trajectory_list)
        #add control trajectory
        trajectory_list.extend(control_trajectory)
        #print(len(control_trajectory))
        #plotting.plot_controller_sit_8_1(trajectory_list, current_time, boundary_list_len, save=False)
        
        sim.plot_constraints_with_control(manipulator.boundary_points,constraint_curve, control_trajectory[0], situation)
        
        #plotting.plot_actuation_level_sit_8_1(actuation_levels, current_time, save=False)
        #plotting.plot_inputs_sit_8_1(u_values, current_time, save=False)
        #plotting.plot_input_vs_x_sit_8_1(u_val_with_state, current_time, save=False)  
        #plotting.plot_actuation_level_vs_x_sit_8_1(L_val_with_state, current_time, save=False) 
        
if __name__ == "__main__":
    start_time =  dt.datetime.now()
    situation = 8.1
    find_ROS(situation, start_time)
    """
    #run all situations to test
    situation=1
    while situation <= 8:
        print("attempting situation ", situation)
        try:
            find_ROS(situation)
            print("situation ", situation, " complete")
        except:
            print ("situation ", situation, " failed")
        
        situation = situation + 1
    """
    #print("1: ", type(1.3))
    #print("2: ", type(["1", ["hjviukvf"]]))
        
    #if isinstance("1.3", float):
    #    print("it worked")

    end_time =  dt.datetime.now()    
    diff = end_time-start_time
    minutes = diff.seconds/60
    print("simulation took ", minutes, " minutes")
    print("all simulations complete")