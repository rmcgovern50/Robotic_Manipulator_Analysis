import sympy
from sympy import symbols

import functions.ROS_simulations_plotting as plotting
import functions.ROS_algorithm_simulations as sim
#import functions.ml_ROS_algorithm_simulations as sim

import datetime as dt
import numpy as np
from my_sorting import combine_to_tuples
from functions.controller import path_dynamics_controller


import save_data as save_data
from scipy.optimize import lsq_linear as lsq_linear

import json

import matplotlib.pyplot as plt
import lsqlin

class simulation:
    def __init__(self, simulation_parameters, things_to_run, GUI_input_data):
        """
        simulation_parameters containts a dictionary, if we have the default loaded example 
        we are dealing with the two DOF manipulator, otherwise it may be a matlab one
        
        things_to_run is a dictionary that can be expned to suit, it essentially 
        informs the simulation which parts can be loaded from data and which are good to go
        """
        self.data = GUI_input_data
        self.current_time =  dt.datetime.now()
        self.robot = simulation_parameters['robot']
        
        del simulation_parameters['robot']
        self.simulation_parameters = simulation_parameters
        self.run = things_to_run

        self.run_simulation()

    def run_simulation(self):
        """
        Create manipulator object this sets up equations for path dynamics and
        performs a range of functions on this        
        """
        self.create_manipulator() # create manipulator object
        """
        implement the additional constraints data, this function performs
        min (natural upper, additional upper)
        """
        self.generate_resultant_constraints()
        """
        now create control object to allow for trajectory generation with the additional
        constraints
        """
        if self.run['step 1'][1] == True:
            self.control = path_dynamics_controller(self.manipulator)
            upper_ad_constraint = self.data.additional_upper_constraint
            if self.data.additional_lower_constraint != "N/A":
                lower_ad_constraint = -1*self.data.additional_lower_constraint
            else:
                lower_ad_constraint = "N/A"
            new_constraint_list = \
                sim.modify_constraints_list(upper_constraint_to_add=upper_ad_constraint,\
                                            lower_constraint_to_add=lower_ad_constraint)

            self.control.set_constraint_list(new_constraint_list)

            #save_data.save_obj(self.control.fast_constraint_matrix, self.simulation_parameters['folder_name'], "fast_contraint_matrix")        
            save_data.save_obj(self.control.very_fast_Ax_Dx_list, self.simulation_parameters['folder_name'], "fast_AxDx")

        else:
            #fast_constraint_matrix = save_data.load_obj(self.simulation_parameters['folder_name'], "fast_contraint_matrix")  
            very_fast_Ax_Dx_list = save_data.load_obj(self.simulation_parameters['folder_name'], "fast_AxDx")   


            self.control = path_dynamics_controller(self.manipulator, form_sped_up_constraints=False)
            #self.control.fast_constraint_matrix = fast_constraint_matrix
            self.control.very_fast_Ax_Dx_list = very_fast_Ax_Dx_list

        self.create_initial_trajectories() #perform first back simulation to the constraints
        
        self.extend_lower()
        self.extend_upper()

    def create_manipulator(self):
        print("Performing step 1")
        #step 1
        self.manipulator = sim.Step_1(self.current_time,\
                                 self.robot,\
                                 self.simulation_parameters,\
                                 run_full_simulation=self.run['step 1'][0],\
                                 save_folder=self.simulation_parameters['folder_name'])
        print("Step 1 complete")
        
    def create_initial_trajectories(self, rough_plot=False):
        print("Performing step 2")
        
        extreme_trajectories,\
        self.xa_intersect,\
        self.xd_intersect,\
        self.perform_step_3,\
        self.perform_step_4 = \
            sim.Step_2(self.current_time,\
                           self.manipulator, self.control,\
                           self.simulation_parameters['target_set'],\
                           self.resultant_upper_constraint_curve_data,\
                           self.resultant_lower_constraint_curve_data,\
                           run_constraint_simulations=self.run['step 2'][0],\
                           run_curve_simulations=self.run['step 2'][1],\
                           run_ROS_boundary_xd_sim=self.run['step 2'][2],\
                           run_ROS_boundary_xa_sim=self.run['step 2'][3],\
                           save_folder=self.simulation_parameters['folder_name'],\
                           work_out_steps_to_run=self.run['step 2'][4])
        
        
        self.initialise_upper_bound_ROS(extreme_trajectories[0])
        self.initialise_lower_bound_ROS(extreme_trajectories[1])
        
        if rough_plot == True:
            
            x1_val = [x[0] for x in extreme_trajectories[0]]
            y1_val = [x[1] for x in extreme_trajectories[0]]   
            plt.plot(x1_val, y1_val, color='purple',label="Td", linewidth=3)
            
            x1_val = [x[0] for x in extreme_trajectories[1]]
            y1_val = [x[1] for x in extreme_trajectories[1]]   
            plt.plot(x1_val, y1_val, color='purple',label="Ta", linewidth=3)        
            
            width_of_lines=2
            if self.data.additional_upper_constraint!= "N/A":
                x1_val = [x[0] for x in self.resultant_upper_constraint_curve_data]
                y1_val = [x[1] for x in self.resultant_upper_constraint_curve_data]   
                plt.plot(x1_val, y1_val, color='green',label="upper resultant", linewidth=width_of_lines)  
                
            if self.data.additional_lower_constraint != "N/A":                
                x1_val = [x[0] for x in self.resultant_lower_constraint_curve_data]
                y1_val = [x[1] for x in self.resultant_lower_constraint_curve_data]   
                plt.plot(x1_val, y1_val, color='green',label="lower resultant", linewidth=width_of_lines)  
            
            
            plt.xlabel("$x_1$")
            plt.ylabel("$x_2$")
            plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
            plt.title("initial_trajectories")
            plt.show()
        
        print("Step 2 complete")
        
    def extend_lower(self):

        if self.perform_step_3:
            print("Performing step 3")

            if self.run['step 3'][0]:
                
                region_to_extend = [self.resultant_lower_constraint_curve_data[0],\
                                    self.xa_intersect]
                lower_boundary_trajectories = self.run_extend_lower(region_to_extend)
                
                self.extend_lower_bound_ROS(lower_boundary_trajectories)
                
                save_data.save_obj(self.lower_bound_ROS, self.simulation_parameters['folder_name'], "step_3_lower_bound_data")        
            else:
                self.lower_bound_ROS = save_data.load_obj(self.simulation_parameters['folder_name'], "step_3_lower_bound_data")
                
            print("Step 3 complete")            

    def extend_upper(self):
        
        if self.perform_step_4:
            print("Performing step 4")

            if self.run['step 4'][0]:

                region_to_extend = [self.resultant_upper_constraint_curve_data[0],\
                                    self.xd_intersect]
                
                upper_boundary_trajectories = self.run_extend_upper(region_to_extend)
    
                self.extend_upper_bound_ROS(upper_boundary_trajectories)
                
                save_data.save_obj(self.upper_bound_ROS, self.simulation_parameters['folder_name'], "step_4_lower_bound_data")        
            else:
                self.upper_bound_ROS = save_data.load_obj(self.simulation_parameters['folder_name'], "step_4_lower_bound_data")
                
                
            print("Step 4 complete")
            
    
    def run_extend_lower(self, region_to_extend):
        print("extending lower bound")
 
        self.sx1_lower_data, _, _ = self.generate_sx_data(self.resultant_lower_constraint_curve_data, boundary="lower")     
        
        interval_list = [(0,0.4), (0.4,1.0)]
        self.lower_sx1_approx_coeff, self.true_lower_interval_list = self.polynomial_approximation_sx(self.sx1_lower_data, intervals=interval_list, order=3)
        actuation_level = 1
        lower_bound_trajectories = sim.extend(self.manipulator,\
                                              self.control,\
                                              region_to_extend,\
                                              self.lower_sx1_approx_coeff,\
                                              self.true_lower_interval_list,\
                                              self.resultant_lower_constraint_curve_data,\
                                              self.data.additional_lower_constraint,\
                                              actuation_level)
        return lower_bound_trajectories

    def run_extend_upper(self, region_to_extend):
        print("extending upper bound")
        
        self.sx1_upper_data, _, _ = self.generate_sx_data(self.resultant_upper_constraint_curve_data, boundary="upper")
        
        interval_list = [(0,0.1), (0.1, 0.2), (0.2,0.3), (0.3, 0.4), (0.4,0.5),\
                         (0.5, 0.6), (0.6,0.7), (0.7, 0.8), (0.8,0.9), (0.9,1.0)] #[(0,1)]
        
        self.upper_sx1_approx_coeff, self.true_upper_interval_list = \
            self.polynomial_approximation_sx(self.sx1_upper_data,\
                                             intervals=interval_list,\
                                             order=3)
        actuation_level = 0
        upper_bound_trajectories = sim.extend(self.manipulator,\
                                              self.control,\
                                              region_to_extend,\
                                              self.upper_sx1_approx_coeff,\
                                              self.true_upper_interval_list,\
                                              self.resultant_upper_constraint_curve_data,\
                                              self.data.additional_upper_constraint ,\
                                              actuation_level)
        print("length of the upper trajectories: ", len(upper_bound_trajectories))
        return upper_bound_trajectories
    
    def initialise_upper_bound_ROS(self, data):
        """
        data - list of tuples [(x1, x2)_1, ... (x1, x2)_N]
        """
        self.upper_bound_ROS = data
        
    def extend_upper_bound_ROS(self, data):
        """
        data - list of tuples [(x1, x2)_1, ... (x1, x2)_N]
        """
        
        self.upper_bound_ROS.extend(data)
        
    def initialise_lower_bound_ROS(self, data):
        """
        data - list of tuples [(x1, x2)_1, ... (x1, x2)_N]
        """
        
        self.lower_bound_ROS = data
        
    def extend_lower_bound_ROS(self, data):        
        """
        data - list of tuples [(x1, x2)_1, ... (x1, x2)_N]
        """
        self.lower_bound_ROS.extend(data)

    def generate_resultant_constraints(self, rough_plot=False):

        self.manipulator_lower_boundary = self.manipulator.lower_boundary_points
        x1, x2\
        = symbols('x1 x2')
        
        ad_up_con = self.data.additional_upper_constraint
        ad_lo_con = self.data.additional_lower_constraint
        
        self.resultant_upper_constraint_curve_data = sim.min_of_constraints(self.manipulator.boundary_points, ad_up_con)
        self.resultant_lower_constraint_curve_data = sim.max_of_constraints(self.manipulator_lower_boundary, ad_lo_con)
        
        if rough_plot:
            x1_val = [x[0] for x in self.resultant_upper_constraint_curve_data]
            y1_val = [x[1] for x in self.resultant_upper_constraint_curve_data]
            
            x1_val2 = [x[0] for x in self.resultant_lower_constraint_curve_data]
            y1_val2 = [x[1] for x in self.resultant_lower_constraint_curve_data]
    
            plt.plot(x1_val, y1_val, color='red')
            plt.plot(x1_val2, y1_val2, color='green')        
            plt.show()

    def generate_sx_data(self, data, boundary="upper"):
        
        sx1_list = [(0,0)]
        sx2_list = [(0,0)]
        sx12_list= [(0,0,0)]
        i = 0
        for state in data:
            
            D, A = self.manipulator.calc_upper_lower_bound_values(state)
            
            x1 = state[0]
            x2 = state [1]
            
            #if we are on the last state, use previous derivative as we cannot get a new one
            if i < (len(data)-1):
                
                x1_next = data[i+1][0]
                x2_next = data[i+1][1]
                gradient = (x2_next - x2)/(x1_next - x1)
            else:
                pass #dont change the gradient variable
            
            if boundary == "upper":
                sx = D - gradient * x2
                
                sx1_list.append((x1, sx))
                sx2_list.append((x2, sx))
                sx12_list.append((x1, x2, sx))
                
            elif boundary == "lower":
                sx = gradient * x2 - A
                
                sx1_list.append((x1, sx))
                sx2_list.append((x2, sx))
                sx12_list.append((x1, x2, sx))
                
            i = i + 1
            
        sx1_list.pop(0)
        sx2_list.pop(0)
        sx12_list.pop(0)
                
        return sx1_list, sx2_list, sx12_list


    def polynomial_approximation_sx(self, data, intervals=[(0,1.0)], order=3, boundary="upper", rough_plot=True):

        i=0
        element_to_check = 0
        
        start_element = 0 
        end_element = 0
        
        data_x_list = [(0,0)]
        data_y_list = [(0,0)]        
        for interval in intervals:
            upper_bound_of_interval = interval[1]
            data_element = data[element_to_check][0]
            
            while data_element < upper_bound_of_interval:
                data_element = data[element_to_check][0]
                element_to_check = element_to_check + 1 
            
            end_element = element_to_check
            
            data_x = [float(x[0]) for x in data[start_element:end_element+1]]
            data_y = [float(x[1]) for x in data[start_element:end_element+1]]
            
            data_x_list.append(data_x)
            data_y_list.append(data_y)
            
            start_element = end_element
            i= i+1
            
        data_x_list.pop(0)
        data_y_list.pop(0)
        
        coeff_list = [0]
        true_interval_list = [(0,0)]
        
        for data_x, data_y in zip(data_x_list, data_y_list):
            coefficients= self.polynomial_approximate_data(data_x, data_y, order)
            coeff_list.append(coefficients)
            true_interval_list.append((data_x[0], data_x[-1]))    
            #print(coefficients)
            
        true_interval_list.pop(0)
        coeff_list.pop(0)
        
        return coeff_list, true_interval_list

    def polynomial_approximate_data(self, data_x, data_y, order, rough_plot=False):
        
        
        input_data = np.array(np.array(data_x), ndmin=2).T
        ones_col = np.ones((len(input_data),1))
        
        input_matrix = ones_col
        
        data_x_st_ed = [data_x[0], data_x[-1]]
        
        start_end_data= np.array(np.array(data_x_st_ed), ndmin=2).T
        
        start_end_matrix = np.ones((2,1))*(start_end_data**0)
        
        #print(start_end_data)
        
        for i in range(1, order+1):
            input_matrix = np.append(input_matrix, input_data**i, axis=1)
            start_end_matrix = np.append(start_end_matrix, start_end_data**i, axis=1)    
        
        Aeq = start_end_matrix
        beq = np.array(np.array([data_y[0], data_y[-1]]), ndmin=1).T
        #print("Aeq = ", Aeq)
        #print("beq = ", beq)        
        
        output_data = np.array(np.array(data_y), ndmin=1).T
        
        C = input_matrix
        d = output_data
        
        #setup the upper bound as the constraint curve
        A = -1*C
        b = -1*d

        #print("C", C, "d", d)        
        #print("A", A, "b", b)
        #print(C, d)
        result = lsqlin.lsqlin(C, d, 0, A, b, Aeq, beq)     
        #print(result['x'].T)
        
        coeff = list(result['x'])
        
        coeff.reverse()
        
        fitted_output = np.polyval(coeff, input_data)#coeff[0] + coeff[1]*input_data + coeff[2]*input_data**2 + coeff[3]*input_data**3
        #print(fitted_output)
        if rough_plot == True:        
            #test the fit
            plt.plot(input_data, output_data, 'or', color='blue', ms=1, label="data")
            plt.plot(input_data, fitted_output, 'or', color='red', ms=1, label="polynomial approx")
            plt.xlabel("$x_1$")
            plt.ylabel("$S(x_1)$")
            plt.legend()
            plt.show()
        
        return coeff
        
    def plot_polynomial_approximation_sx(self, boundary="upper"):
        
        if boundary=="upper":
            coefficient_list = self.upper_sx1_approx_coeff
            interval_list = self.true_upper_interval_list
            data = self.sx1_upper_data
        elif boundary=="lower":
            coefficient_list = self.lower_sx1_approx_coeff
            interval_list = self.true_lower_interval_list
            data = self.sx1_lower_data
        
        seg_number = 1
        for coeff, interval in zip(coefficient_list, interval_list):
            data_x = np.linspace(interval[0],interval[1],num=100)
            data_y = np.polyval(coeff, data_x)        
            plt.plot(data_x, data_y, 'or', color='red', ms=1, label="polynomial approx" + str(seg_number))
            seg_number = seg_number + 1

                
            
        x1_val = [x[0] for x in data]
        x2_val = [x[1] for x in data]
        
        plt.plot(x1_val, x2_val, 'or', color='blue', ms=1, label="data")
        #plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        #plt.legend()
        plt.xlabel("$x_1$")
        plt.ylabel("$S(x_1)$")
        plt.grid()
        plt.show()
        

    def plot_sx1(self, boundary="upper"):

        if boundary == "upper" or boundary == "both":
            
            x1_val = [x[0] for x in self.sx1_upper_data]
            sx_val = [x[1] for x in self.sx1_upper_data]
            plt.plot(x1_val, sx_val,color='red', label="$s(x_1)$ upper")
        
        if boundary == "lower" or boundary == "both":
            x1_val = [x[0] for x in self.sx1_lower_data]
            sx_val = [x[1] for x in self.sx1_lower_data]
            plt.plot(x1_val, sx_val,color='blue', label="$s(x_1)$ lower")
        
        plt.title("$s(x_1) data plot")
        plt.xlabel("$x_1$")
        plt.ylabel("$s(x_1)$")
        plt.grid()
        #plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        #plt.legend()
        plt.show()            
        

    def plot_constraints(self):
        """
        soft_constraint = sim.generate_arbitrary_constraint_set_data(self.situation) 
        
        plotting.plot_constraints(self.manipulator.boundary_points,\
                                      soft_constraint,\
                                      self.current_time,\
                                      save_file=False)
        """
        width_of_lines = 3.0
        
        
        natural_upper_constraint = self.manipulator.boundary_points
        x1_val = [x[0] for x in natural_upper_constraint]
        y1_val = [x[1] for x in natural_upper_constraint]
        plt.plot(x1_val, y1_val, color='red', label="upper natural", linewidth=width_of_lines)
        
        nautural_lower_constraint = self.manipulator_lower_boundary
        x1_val = [x[0] for x in nautural_lower_constraint]
        y1_val = [x[1] for x in nautural_lower_constraint]
        plt.plot(x1_val, y1_val, color='red', label="lower natural", linewidth=width_of_lines)
        
        if self.data.additional_upper_constraint!= "N/A":
            
            additional_upper_constraint = sim.evaluate_expression(self.data.additional_upper_constraint)
            x1_val = [x[0] for x in additional_upper_constraint]
            y1_val = [x[1] for x in additional_upper_constraint]      
            plt.plot(x1_val, y1_val, color='orange', linestyle="dashed",label="upper additional", linewidth=width_of_lines)  
            
            x1_val = [x[0] for x in self.resultant_upper_constraint_curve_data]
            y1_val = [x[1] for x in self.resultant_upper_constraint_curve_data]   
            plt.plot(x1_val, y1_val, color='green', linestyle="dotted",label="upper resultant", linewidth=width_of_lines)  
            


        if self.data.additional_lower_constraint != "N/A":
            additional_lower_constraint = sim.evaluate_expression(self.data.additional_lower_constraint)       
            x1_val = [x[0] for x in additional_lower_constraint]
            y1_val = [x[1] for x in additional_lower_constraint]
            plt.plot(x1_val, y1_val, color='orange', linestyle="dashed", label="lower additional", linewidth=width_of_lines)
            
            x1_val = [x[0] for x in self.resultant_lower_constraint_curve_data]
            y1_val = [x[1] for x in self.resultant_lower_constraint_curve_data]   
            plt.plot(x1_val, y1_val, color='green', linestyle="dotted",label="lower resultant", linewidth=width_of_lines)  
        
        
        plt.xlabel("$x_1$")
        plt.ylabel("$x_2$")
        plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.grid()
        plt.show()
        
    def plot_ROS_bounds_with_constraints(self):
        try:
            x1_val = [x[0] for x in self.upper_bound_ROS]
            y1_val = [x[1] for x in self.upper_bound_ROS]  
            
            zipped_lists = zip(x1_val, y1_val)
            sorted_pairs = sorted(zipped_lists)
            tuples = zip(*sorted_pairs)
            x1_val, y1_val = [ list(tupl) for tupl in tuples]
            
            plt.plot(x1_val, y1_val, color='green',label="Td", linewidth=2.5)
        except:
            print("Can't plot upper bound")
            
        try:
            x1_val = [x[0] for x in self.lower_bound_ROS]
            y1_val = [x[1] for x in self.lower_bound_ROS]   
            
            zipped_lists = zip(x1_val, y1_val)
            sorted_pairs = sorted(zipped_lists)
            tuples = zip(*sorted_pairs)
            x1_val, y1_val = [ list(tupl) for tupl in tuples]
            
            plt.plot(x1_val, y1_val, color='green',label="Ta", linewidth=2.5)        
        except:
            print("Can't plot lower bound")
            
        width_of_lines=1
        if self.data.additional_upper_constraint!= "N/A":
            x1_val = [x[0] for x in self.resultant_upper_constraint_curve_data]
            y1_val = [x[1] for x in self.resultant_upper_constraint_curve_data] 
            
            zipped_lists = zip(x1_val, y1_val)
            sorted_pairs = sorted(zipped_lists)
            tuples = zip(*sorted_pairs)
            x1_val, y1_val = [ list(tupl) for tupl in tuples]
            
            plt.plot(x1_val, y1_val, color='red',label="upper resultant", linewidth=width_of_lines)  
            


        if self.data.additional_lower_constraint != "N/A":                
            x1_val = [x[0] for x in self.resultant_lower_constraint_curve_data]
            y1_val = [x[1] for x in self.resultant_lower_constraint_curve_data]
            
            zipped_lists = zip(x1_val, y1_val)
            sorted_pairs = sorted(zipped_lists)
            tuples = zip(*sorted_pairs)
            x1_val, y1_val = [ list(tupl) for tupl in tuples]            
            
            plt.plot(x1_val, y1_val, color='red',label="lower resultant", linewidth=width_of_lines)  
                
        plt.xlabel("$x_1$")
        plt.ylabel("$x_2$")
        #plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.title("initial_trajectories")
        plt.grid()
        plt.show()
    