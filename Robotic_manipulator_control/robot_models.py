# -*- coding: utf-8 -*-
"""
This is a file that will contain models for specific robots I wish to study
Each robot, robot specific parts will be coded here
These are all child classes the the path_dynamics_analysis class which helps to 
visualise the state space in a general way
"""

from sympy import symbols, Matrix, sin, cos, acos, asin, atan2, sqrt, pprint, diff

from path_dynamics_analysis import path_dynamics as pd
#from path_dynamics_control import path_dynamics_controller as pdc
from robot_data_visualisation import two_dof_robot_data_visualisation as rdv

import math as m
import my_math as mm
import numpy as np
import datetime as dt

#from my_visualising import simple_plot
import my_visualising as mv
import my_sorting as ms

import matplotlib.pyplot as plt


class two_dof_planar_robot(pd):
    pass
    """
    This class is for the case specific parts of revolute slide robot
    The path_dynamics analysis class will use the information here to
    calculate admissable regions of the state space along a path
    """
    
    def __init__(self, robot_parameters, current_time):
        """
        Initialise the robot with it's parameters
        Arguments: 
        end_effector_mass (mass of end effector in kg)
        joint_limits list of tuples describing max joint limits of form
        [(u1min,u1max), (u2min,u2max), ... , (unmin,unmax)]
        the joint limits will always be a list of 2 tuples in this case
        """
        self.type = "revolute prismatic robot (polar robot)"
        self.link_lengths = robot_parameters['link_lengths']
        self.joint_masses = robot_parameters['joint_masses']
        self.joint_limits = robot_parameters['actuator_limits']
        
        #just assume the x1_axis is a lower constraint
        nat_lower_boundary_x1 = np.linspace(0,1,num=100)
        nat_lower_boundary_x2 = np.linspace(0,0,num=100)
        self.lower_boundary_points = ms.combine_to_tuples(nat_lower_boundary_x1, nat_lower_boundary_x2)
        
        
        joint_mass = self.joint_masses
        link_lengths = self.link_lengths
        joint_limits = self.joint_limits


        self.s, self.sd, self.sdd = symbols('s sd sdd')

        self.m1, self.m2, self.l1, self.l2, self.u1min, self.u1max, self.u2min, self.u2max,\
        = symbols('m1, m2 L1 L2 u1min u1max, u2min u2max')#make symbols for all the varables for symbolic calculations
        
        self.q1, self.q2, self.q1d, self.q2d, self.q1dd, self.q2dd, self.u1, self.u2 \
        = symbols('q1 q2 q1d q2d q1dd q2dd u1, u2')
        
        #list that is of the form that allows easy substitution of robot parameters.
        symbols_to_sub = [self.m1, self.m2, self.l1, self.l2, self.u1min, self.u1max, self.u2min, self.u2max]
        values_to_sub = [joint_mass[0], joint_mass[1], link_lengths[0], link_lengths[1], joint_limits[0][0], joint_limits[0][1], joint_limits[1][0], joint_limits[1][1]]
        
        self.constants_to_sub = ms.combine_to_tuples(symbols_to_sub, values_to_sub)
        self.dynamics()
        
        #super(two_dof_planar_robot, self).__init__(constants_to_sub, self.s, self.sd, self.sdd, self.qd)
        pd. __init__(pd, self.M, self.C, self.g, self.q1, self.q2, \
                     self.q1d, self.q2d, self.q1dd, self.q2dd,self.constants_to_sub,\
                     self.s, self.sd, self.sdd, self.qd,)
        
    def dynamics(self):
        
        """
        Form the dynamical equations in matrix form, return the result
        """
        #define moments of inertia              
        #define inertia matrix
        
        m1 = self.m1
        m2 = self.m2
        l1 = self.l1
        l2 = self.l2
        q1 = self.q1
        q2 = self.q2
        q1d = self.q1d
        q2d = self.q2d
        q1dd = self.q1dd
        q2dd = self.q2dd
        
        
        #there is a serious mistake as q1 and q2 are defined the wrong way around
        g = 9.81 #ms^-2
        
        M11 = m1*(l1**2) + m2*(l1**2 + 2*l1*l2*cos(q2)+ l2**2)  
        M12 = m2*(l1*l2*cos(q2) + l2**2)        
        M21 = m2*(l1*l2*cos(q2) + l2**2)        
        M22 = m2*l2**2
        
        self.M = Matrix([[M11, M12], [M21, M22]])#form inertia matrix
        
        C11 = 0
        C12 = (-1*m2*l1*l2*sin(q2)*(2*q1d + q2d))
        C21 = (m2*l1*l2*q1d*sin(q2))  
        C22 = 0
        self.C = Matrix([[C11, C12],[C21, C22]])
        
        G1 = (m1+m2)*l1*g*cos(q1) + m2*g*l2*cos(q1+q2)
        G2 = m2*g*l2*cos(q1+q2)
        
        self.g = Matrix([[G1], [G2]])
        
        self.q = Matrix([[self.q1],[self.q2]])
        
        self.qd = Matrix([[self.q1d], [self.q2d]])

        self.qdd = Matrix([[self.q1dd], [self.q2dd]])
        
        self.inputs = Matrix([[self.u1], [self.u2]])
        
        #pprint(self.C * self.qd)
        return self.M, self.C, self.g
    
    
    
    def calc_upper_lower_bound_values(self, point):
        """
        Method to take in a point and evaluate the upper and lower bounds at that point
        Arguments:
            point = (s, sd)
        """
        L = point[0]
        U = point[1]
        bounds = self.bounds
        i = 0
        
        for bound in bounds:
            
            lim_direction_decider = bound[2].subs([(self.s, point[0]), (self.sd, point[1])])
            sub_list = [(self.s, point[0]), (self.sd, point[1])]
            
            #put the bounds the right way round
            if lim_direction_decider > 0:
                L_to_check =  bound[0].subs(sub_list)
                U_to_check = bound[1].subs(sub_list)
              
            elif lim_direction_decider < 0:
                L_to_check =  bound[1].subs(sub_list)
                U_to_check = bound[0].subs(sub_list)
                
            else:
                raise Exception("M(s) cannot be equal to zero - error in calc_upper_lower_bound_values method")
            
            if i == 0:
                L = L_to_check
                U = U_to_check
            else:
                if L_to_check > L:
                    L = L_to_check
                if U_to_check < U:
                    U = U_to_check
            i = i + 1
                       
        return L, U
    
    
    def end_effector_Jacobian(self):
        """
            Simple methodf for evaluating the Jacobian of the end effector of the manipulator
            \dot{x} = J \dot{q}
            
            return -
            J = 3x2 matrix describing the non linear transformation from joint 
                velocities to end effectors linear velocity
        """
    
        l1 = self.l1
        l2 = self.l2   
        q1 = self.q1
        q2 = self.q2
        
        self.J_linear = Matrix([[-l1*sin(q1) - l2*sin(q1+q2) , -l2*sin(q1+q2)]\
                                ,[l1*cos(q1) + l2*cos(q1+q2),  l2*cos(q1 + q2)]\
                                ,[0,0]])
        return self.J_linear
    
    def forward_kinematics(self, joints):
        """
        Arguments:
            function takes in list of joint positions of the form:
            joints = [q1, q2]
            q1 in radians, q2 in radians
        
        returns:
            end effector pose
            pose = [x, y]
            x in length units, y in length units
        """
        q1 = joints[0]
        q2 = joints[1]
        l1 = self.link_lengths[0]
        l2 = self.link_lengths[1]        
        
        
        x = l1*m.cos(q1) + l2*m.cos(q1+q2)
        y = l1*m.sin(q1) + l2*m.sin(q1+q2)
        
        pose = [x, y]
        
        return pose
    def inverse_kinematics(self, pose, return_degrees=0):
        """
        Arguments:
            function takes in end effector pose
            pose = [x, y]
            return_degrees if set to 1 returns q1 as degrees instead of radians
        (model taken from park, modern robotics page 220)
            
        return:
            joint positions as
            joint_positions = [q1, q2]
            where q1 is in radians, q2 in length units
        """
        L1 = self.link_lengths[0]
        L2 = self.link_lengths[1]
        x = pose[0]
        y= pose[1]
        
        beta = m.acos((L1**2+L2**2-x**2-y**2)/(2*L1*L2))
        alpha = m.acos((x**2+y**2+L1**2-L2**2)/(2*L1*m.sqrt(x**2+y**2)))
        gamma = m.atan2(y,x)
            
        q1 = gamma - alpha
        q2 = m.pi - beta



        joint_positions = [q1,q2]
        return joint_positions
    
    
    def symbolic_inverse_kinematics(self, pose, return_degrees=0):
        """
        Arguments:
            function takes in end effector pose
            pose = [x, y]
            return_degrees if set to 1 returns q1 as degrees instead of radians
        (model taken from park, modern robotics page 220)
            
        return:
            joint positions as
            joint_positions = [q1, q2]
            where q1 is in radians, q2 in length units
        """
        L1 = self.link_lengths[0]
        L2 = self.link_lengths[1]
        x = pose[0]
        y= pose[1]

        beta = acos((L1**2+L2**2-x**2-y**2)/(2*L1*L2))
        alpha = acos((x**2+y**2+L1**2-L2**2)/(2*L1*sqrt(x**2+y**2)))
        gamma = atan2(y,x)
            
        q1 = gamma - alpha
        q2 = m.pi - beta
    
    
    
        joint_positions = [q1,q2]
        return joint_positions
    
    
    def straight_line_parameterisation(self, start, end):
        """
        Arguments:
            start = [x1, y1]
            end = [x2, y2]
            
        return:
            qs = [q1(s), q2(s)]
            
        Description:
            X(s) = Xstart + s*(Xend - Xstart)
               where X(s) = [x, y] = [xstart + s*(xend - xstart), ystart + s*(yend - ystart)  
            
        """
        xstart = start[0]
        xend = end[0]
        ystart = start[1]
        yend = end[1]
        
        xs = xstart + self.s*(xend - xstart)
        ys = ystart + self.s*(yend - ystart)
        
        Xs = [xs, ys]
        qs = self.symbolic_inverse_kinematics(Xs)
        
        return qs
        
    def joint_space_straight_line_parameterisation(self, start, end):
        """
        Arguments:
            start = (q1start, q2start)
            end = (q1end, qend)
            
        return:
            qs = [q1(s), q2(s)]
        """
        #print(start)
        #print(end)
        q1s = start[0] - self.s*(start[0] - end[0])
        q2s = start[1] - self.s*(start[1] - end[1])   
        
        qs = [q1s, q2s]
        
        return qs
        
    def circular_arc_parameterisation(self, centre = (0,0.3), R=0.1):
        """
        Parameters
        ----------
        centre : centre point of the circle
            [xc, yc]
        Returns
        -------
        qs : symbolic representation of the parameterisation
            [q1(s), q2(s)]
        """
        
        xc = centre[0]
        yc = centre[1]
        print(xc, R )
        xs = xc + R*cos(self.s*m.pi)
        ys = yc - R*sin(self.s*m.pi)        
        
        Xs = [xs, ys]
        qs = self.symbolic_inverse_kinematics(Xs)
        
        return qs
        
    def simulate_trajectory(self, qs, increment=0.1):
        """
        Arguments:
            qs - list of parameterised trajectories
            increment - set increment size 
        return:
        """
        s = 0
        coordinates = []
        s_axisq1 = []
        s_axisq2 = []
        q1_v_q2 = []
        i = 0
        while(s < 1):
            if s == 0:
                self.coordinates_q1_q2 = [(qs[0].subs([(self.s, s)]), qs[1].subs([(self.s, s)]))]
                #print(coordinates)
                x_y = self.forward_kinematics([self.coordinates_q1_q2[0][0],self.coordinates_q1_q2[0][1]])
                self.x_y_coordinates = [(x_y[0], x_y[1])]
                
                self.s_axisq1 = [(s, qs[0].subs([(self.s, s)]))]
                self.s_axisq2 = [(s, qs[1].subs([(self.s, s)]),s)]
            else:
                self.coordinates_q1_q2.append((qs[0].subs([(self.s, s)]), qs[1].subs([(self.s, s)])))
                #print(coordinates)
                x_y = self.forward_kinematics([self.coordinates_q1_q2[i][0],self.coordinates_q1_q2[i][1]])
                self.x_y_coordinates.append((x_y[0], x_y[1]))
                
                self.s_axisq1.append((s, qs[0].subs([(self.s, s)])))
                self.s_axisq2.append((s, qs[1].subs([(self.s, s)])))
            s = s + increment
            i = i + 1


    def get_direction_unit_vector(self, line_definition):
        """
        Parameters
        ----------
        line_definition : list
            [(x1, y1), (x2, y2)] = [(start_coordinate), (end_coordinate)]

        Returns
        -------
        [xdir, ydir, zdir]
        unit vector pointing from start to end
        """
        
        x1= line_definition[0][0]
        x2 = line_definition[1][0]
        y1= line_definition[0][1]
        y2 = line_definition[1][1]        
        
        
        distance = [x2 - x1, y2 - y1]
        length = m.sqrt(distance[0]**2 + distance[1]**2)
        direction = [distance[0]/length, distance[1]/length]
        
        return direction
        
    def run_full_path_dynamics_analysis(self, path_def, s_lims, sd_lims):
        """
        Description-
            This is a method that uses many of the existing methods in this class
            to simply produce an admissable region plot amoung other information
            that can then be used to design controllers in this space
        
        Arguments:
            path_def - ["path type", parameters] - point to point movement coordinates
            s_lims - [start s, end s, s increment] - sampling rate for admissable region
            sd_lims - [start sd, end sd, sd increment] - sampling rate for admissable region
        
        return:
            region - list of tuples decribing the admissable region
            boundry_points - list of points decribing the boundary
            boundary expressions - list of expressions that when evaluated can get us the upper and lower limits on sdd
            plt - return plot of the admissable region
        """
        
        #convert q to q(s)
        path_type = path_def[0]
        path_parameters = path_def[1]
        #print(path_type, path_parameters)
        if path_type == "circular arc":
            centre = path_parameters[0]
            radius = path_parameters[1]
            self.qs = self.circular_arc_parameterisation(centre, radius)
        elif path_type == "straight line":
            self.qs = self.straight_line_parameterisation(path_parameters[0], path_parameters[1])
        elif path_type == "joint space line":
            self.qs = self.joint_space_straight_line_parameterisation(path_parameters[0], path_parameters[1])
        else:
            print("error no qs defined")
            
        #self.plot_end_effector_trajectory(qs, 0.01, 1, 1, 1, 1)
        #try:
        #calculate the derivatives
        _, self.qds, self.qdds, dqds, d2qds2  = pd.path_parameterisation(self, self.qs)
        pd.pass_qs_qds_qdds(pd, self.qs, self.qds, self.qdds)
        
        #get all the necessary matrices in terms of the parameterised matrices
        self.Mqs, self.Cqs, self.gqs = pd.calc_qs_matrices(self, self.qs, self.qds, self.qdds)
        print("matrices got")
        
        #form M(s), C(s), and g(s) as M(s)*sdd + C(s)*sd**2 + g(s) = t(s)
        self.Ms, self.Cs, self.gs = pd.calc_s_matrices(self, self.Mqs, self.Cqs, self.gqs, dqds, d2qds2)
        print("vectors got")
        
        #calculate bounds based on the path dynamics
        self.bounds = pd.calc_bounds(self, self.Ms, self.Cs, self.gs)
        print("bounds got")

        self.admissible_region, self.boundary_points = pd.calc_admissable(self ,self.bounds, s_lims, sd_lims)
        print("region got")
        self.s_lims = s_lims
        self.sd_lims = sd_lims
        #except:
            #print("could not run simulation")
        #self.set_simulation_data()

    def evaluate_joint_torques(self, inputs, states):

        Ms = self.Ms
        Cs = self.Cs
        gs = self.gs
        s = self.s
        sd = self.sd
        sdd = self.sdd
        
        
        #print(sympy.shape(Ms))
        tau = Ms*sdd + Cs*sd*sd + gs
        print("shape is: ", tau.shape, tau)
        
        tau_1 = tau.row(0)
        tau_2 = tau.row(1)
        
        torque_vector = []
        
        for state, u in zip(states, inputs):
            x1 = state[0]
            x2 = state[1]

            tau_1_val = tau_1.subs([(s, x1), (sd, x2), (sdd, u)])
            tau_2_val = tau_2.subs([(s, x1), (sd, x2), (sdd, u)])
            print(tau_1_val[0], tau_2_val[0])
            
            if len(torque_vector) == 0:
                torque_vector = [(tau_1_val[0], tau_2_val[0])]
                tau_1_val_with_state = [(tau_1_val[0], state)]
                tau_2_val_with_state = [(tau_2_val[0], state)]
                torque_vector_with_state = [(tau_1_val[0], tau_2_val[0], state)]
            else:
                torque_vector.append((tau_1_val[0], tau_2_val[0]))
                tau_1_val_with_state.append((tau_1_val[0], state))
                tau_2_val_with_state.append((tau_2_val[0], state))
                torque_vector_with_state.append((tau_1_val[0], tau_2_val[0], state))
        #print("torques are", tau)
                
                
        return torque_vector, tau_1_val_with_state, tau_2_val_with_state, torque_vector_with_state

    def get_potential_collision_energy(self, J, M, direction, s_sd):
        """
            method to get the potential collision energy of the manipulator
            Arguments:
                J - jacobian of motion
                M - mass matrix
                direction - []driection between robot velocity and object
                s_sd - list of points the robot passes through 
                        [(s1, sd1), ..., (sn, sdn)]
            return:
                energies list with corresponding s and sd point attacted
                [(s1, sd1, e1), ..., (sn, sdn, en)]                
        """

        n = Matrix(direction)
        energy_list = pd.evaluate_potential_collision_energy(self, J ,M, n, s_sd)
        
        return energy_list

    def set_trajectory_energy_list(self, energy_list):
        self.energy_list = energy_list

    def set_admissible_region_collision_energies(self, energy_list):
        self.admissible_region_collision_energies = energy_list

    def set_simulation_data(self):
        """
        Method to save all the robot data in a dictionary so give a standard format that can be populated 
        by the saved data rather than rerunning the simulation each time
        
        DISCONTINUED METHOD
        
        Returns
        -------
        data : Dictionary
            All the data that needs to be saved
        """
        
        data = {'admissible_region' : self.admissible_region,\
                'admissible_region_grid': [self.s_lims, self.sd_lims],\
                'boundary points': self.boundary_points,\
                'admissible_region_collision': self.admissible_region_collision_energies,\
                'q1_vs_s': self.s_axisq1,\
                'q2_vs_s': self.s_axisq2,\
                'q1_vs_q2': self.coordinates_q1_q2,\
                'xy_coordinates': self.x_y_coordinates,\
                'potential_collision_energy' :  self.energy_list,\
                'link_lengths' : self.link_lengths,\
                'joint_masses' : self.joint_masses,\
                'actuator_limits' : self.joint_limits,\
                'actuator_bounds' : self.bounds\
                }
        self.sim_data = data

        return data
        
    def check_if_model_lipschitz(self):
        
        pd.lipschitz_test_admissible_region(self)
        
    def plot_workspace_movement(self, save=False, plot_start_end_points=True):
        
        try:
            current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots     
            plotter = rdv(current_time)
            plotter.plot_robot_motion_x_y(self.link_lengths, \
                                      self.x_y_coordinates, \
                                      self.s_axisq1, \
                                      start_end_only=plot_start_end_points,\
                                      save=save)
        except:
            print("That didn't work, was the simulation run?")
                
    def plot_admissible_region(self):
        try:
            current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots     
            plotter = rdv(current_time)
            plotter.generate_state_space_plot(self.admissible_region,save=False)
        except:
            print("That didn't work, was the simulation run?")
        
    def plot_joint_space(self, ms=1,save_file=False):

        try:
            current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots     
            plotter = rdv(current_time)
            plotter.plot_q1_against_q2(self.coordinates_q1_q2, save=save_file,\
                               marker_size=ms,\
                               file_name="q2 vs q2",\
                               filepath="paper_plots/")
        except:
            print("That didn't work, was the simulation run?")

    def plot_velocity_limit_curve(self, save_file= False):
        #try:
        current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots     
        plotter = rdv(current_time)

        plotter.generate_velocity_limit_curve_plot(self.boundary_points, save=save_file,\
                           marker_size=1,\
                           file_name="Velocity limit curve",\
                           filepath="paper_plots/")
        #except:
            #print("That didn't work, was the simulation run?")        
