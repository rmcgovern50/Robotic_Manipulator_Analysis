# -*- coding: utf-8 -*-
"""
This is a file that will contain models for specific robots I wish to study
Each robot, robot specific parts will be coded here
These are all child classes the the path_dynamics_analysis class which helps to 
visualise the state space in a general way
"""

from sympy import symbols, Matrix, sin, cos, pprint


from path_dynamics_analysis import path_dynamics as pd
from path_dynamics_control import path_dynamics_controller as pdc

import math as m
import numpy as np

#from my_visualising import simple_plot
import my_visualising as mv
import my_sorting as ms

import matplotlib.pyplot as plt


class two_dof_planar_prismatic(pd, pdc):
    pass
    """
    This class is for the case specific parts of revolute slide robot
    The path_dynamics analysis class will use the information here to
    calculate admissable regions of the state space along a path
    """
    
    def __init__(self, joint_mass, link_lengths, joint_limits):
        """
        Initialise the robot with it's parameters
        Arguments: 
        end_effector_mass (mass of end effector in kg)
        joint_limits list of tuples describing max joint limits of form
        [(u1min,u1max), (u2min,u2max), ... , (unmin,unmax)]
        the joint limits will always be a list of 2 tuples in this case
        """
        self.type = "revolute prismatic robot (polar robot)"
        self.link_lengths = link_lengths
        
        
        self.s, self.sd, self.sdd = symbols('s sd sdd')
        self.joint_limits = joint_limits

        self.m1, self.m2, self.l1, self.l2, self.u1min, self.u1max, self.u2min, self.u2max,\
        = symbols('m1, m2 L1 L2 u1min u1max, u2min u2max')#make symbols for all the varables for symbolic calculations
        
        self.q1, self.q2, self.q1d, self.q2d, self.q1dd, self.q2dd, self.u1, self.u2 \
        = symbols('q1 q2 q1d q2d q1dd q2dd u1, u2')
        
        #list that is of the form that allows easy substitution of robot parameters.
        symbols_to_sub = [self.m1, self.m2, self.l1, self.l2, self.u1min, self.u1max, self.u2min, self.u2max]
        values_to_sub = [joint_mass[0], joint_mass[1], link_lengths[0], link_lengths[1], joint_limits[0][0], joint_limits[0][1], joint_limits[1][0], joint_limits[1][1]]
        
        constants_to_sub = ms.combine_to_tuples(symbols_to_sub, values_to_sub)
        self.dynamics()
        
        super(two_dof_planar_prismatic, self).__init__(constants_to_sub)
        
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
    
    def check_if_dynamics_valid(self):
        
        pprint(self.M)
        print("=======================")
        
        Matrix1 = self.M - 2*self.C
        pprint(Matrix1)
        print("=======================")
        
        
        Matrix1transpose = Matrix1.transpose()
        
        pprint(Matrix1 + Matrix1transpose)
    
    
    
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
    
    def straight_line_parameterisation(self, start, end):
        """
        Arguments:
            start = [x1, y1]
            end = [x2, y2]
            
        return:
            qs = [q1(s), q2(s)]
            
        Description:
        Function works by taking in two points in the cartesian space
        the equation of a straight line is:
            y = mx + c in cartesian space
        This is a plar manipulator so we can define the following transormation for any point in space
        y = q2sinq1
        x = q2cosq1
        
        so a line can be defined as:
            
        q2*cosq1 = q2*m*sinq1 + c
        or:
        q2 = c/(cosq1 - msinq1)
        
        WARNING, this equation will only work correctly first quadrant:
            -
        as 
        
        """

        #get m
        pass
        #return qs
        
        
    def joint_space_straight_line(self, start, end):
        """
        Arguments:
            start = (q1start, q2start)
            end = (q1end, qend)
            
        return:
            qs = [q1(s), q2(s)]
        """
        print(start)
        print(end)
        q1s = start[0] - self.s*(start[0] - end[0])
        q2s = start[1] - self.s*(start[1] - end[1])   
        
        qs = [q1s, q2s]
        print(qs)
        return qs
        
    def plot_end_effector_trajectory(self, qs, increment=0.1,\
                                     plot_q1_against_s=0, plot_q2_against_s=0,\
                                     plot_q1_against_q2=0, plot_end_effector_x_y=0):
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
                coordinates = [(qs[0].subs([(self.s, s)]), qs[1].subs([(self.s, s)]))]
                #print(coordinates)
                x_y = self.forward_kinematics([coordinates[0][0],coordinates[0][1]])
                x_y_coordinates = [(x_y[0], x_y[1])]
                
                s_axisq1 = [(s, qs[0].subs([(self.s, s)]))]
                s_axisq2 = [(s, qs[1].subs([(self.s, s)]),s)]
            else:
                coordinates.append((qs[0].subs([(self.s, s)]), qs[1].subs([(self.s, s)])))
                #print(coordinates)
                x_y = self.forward_kinematics([coordinates[i][0],coordinates[i][1]])
                x_y_coordinates.append((x_y[0], x_y[1]))
                
                s_axisq1.append((s, qs[0].subs([(self.s, s)])))
                s_axisq2.append((s, qs[1].subs([(self.s, s)])))
            s = s + increment
            i = i + 1
        #print(coordinates)
        
        if plot_q1_against_s == 1:
            x_val = [x[0] for x in s_axisq1]
            y_val = [x[1] for x in s_axisq1]

            y_val = np.rad2deg(np.array(y_val, dtype=np.float32))
            plt.plot(x_val, y_val,'or',ms=5)
            
            plt.grid(color='black', linestyle='-', linewidth=0.5)
            plt.title("Angle of q1 vs s")
            plt.xlabel("s")
            plt.ylabel("q1 (degrees)")
            plt.show()
        if plot_q2_against_s == 1:
            x_val = [x[0] for x in s_axisq2]
            y_val = [x[1] for x in s_axisq2]
            
            y_val = np.rad2deg(np.asarray(y_val, dtype=np.float32))
            plt.plot(x_val,y_val,'or',ms=5)
            
            plt.grid(color='black', linestyle='-', linewidth=0.5)
            plt.title("Angle of q2 vs s")
            plt.xlabel("s")
            plt.ylabel("q2 (degrees)")
            plt.show()  
        if plot_q1_against_q2 == 1:
            x_val = [x[0] for x in coordinates]
            y_val = [x[1] for x in coordinates]
            
            x_val = np.rad2deg(np.asarray(x_val, dtype=np.float32))
            y_val = np.rad2deg(np.asarray(y_val, dtype=np.float32))
            
            plt.plot(x_val,y_val,'or',ms=5)
            plt.grid(color='black', linestyle='-', linewidth=0.5)
            plt.title("Angle of q1 vs angle of q2")
            plt.xlabel("q1 (degrees)")
            plt.ylabel("q2 (degrees)")
            plt.show()
        if plot_end_effector_x_y == 1:
            x_val = [x[0] for x in x_y_coordinates]
            y_val = [x[1] for x in x_y_coordinates]
            
            plt.plot(x_val,y_val,'or',ms=5)
            
            plt.grid(color='black', linestyle='-', linewidth=0.5)
            
            max_extension = self.link_lengths[0] + self.link_lengths[1] + 0.2
            plt.xlim(right=max_extension) #xmax is your value
            plt.xlim(left=-max_extension) #xmin is your value
            plt.ylim(top=max_extension) #ymax is your value
            plt.ylim(bottom=-max_extension) #ymin is your value
            
            plt.title("Workspace_trajectory")
            plt.xlabel("x (metres)")
            plt.ylabel("y (metres)")
            plt.show()
            

    def run_full_path_dynamics_analysis(self, path_straight_line, s_lims, sd_lims):
        """
        Description-
            This is a method that uses many of the existing methods in this class
            to simply produce an admissable region plot amoung other information
            that can then be used to design controllers in this space
        
        Arguments:
            path_straight_line - [(xstart, ystart), (xend, yend)] - point to point movement coordinates
            s_lims - [start s, end s, s increment] - sampling rate for admissable region
            sd_lims - [start sd, end sd, sd increment] - sampling rate for admissable region
        
        return:
            region - list of tuples decribing the admissable region
            boundry_points - list of points decribing the boundry
            boundry expressions - list of expressions that when evaluated can get us the upper and lower limits on sdd
            plt - return plot of the admissable region
        """
        
     
        #repeat code used externally except simple_plot
        print(path_straight_line)
        self.qs = self.joint_space_straight_line(path_straight_line[0], path_straight_line[1])#straight_line_parameterisation(path_straight_line[0], path_straight_line[1])
        
        #self.plot_end_effector_trajectory(qs, 0.01, 1, 1, 1, 1)
        
        q, qd,qdd, dqds, d2qds2  = pd.path_parameterisation(self, self.qs)
        #print(q, qd,qdd, dqds, d2qds2)

        #get all the necessary matrices in terms of the parameterised matrices
        Mqs, Cqs, gqs = pd.calc_qs_matrices(self, q, qd, qdd)
        #print(Mqs, Cqs, gqs)
        
        #form M(s), C(s), and g(s) as M(s)*sdd + C(s)*sd**2 + g(s) = t(s)
        Ms, Cs, gs = pd.calc_s_matrices(self, Mqs, Cqs, gqs, dqds, d2qds2)
        #print(Ms, Cs, gs)
        
        #calculate bounds based on the path dynamics
        self.bounds = pd.calc_bounds(self, Ms, Cs, gs)
        #print(bounds)
                #define grid
        #s_lim = [0, 1, 0.1]
        #sd_lim = [0,10, 0.1]
        self.admissable_region, boundry_points = self.calc_admissable(self.bounds, s_lims, sd_lims)

        
        #pd.generate_state_space_plot(self, self.admissable_region)
        
        #get big list of tangent cones corresponding to ead point in the state space 
        

        #tangent_cone = f(s, sd)
    def generate_time_optimal_trajectory(self, plot_trajectory=False):
        """
        method to take steps neccesary to design a time opeitmal control trajectory
        
        return:
            trajectory - [(s1, sd1),...,(sn,sdn) ]
        """

        #calculate admissable region
        #admissable_region, boundry_points = self.calc_admissable(self.bounds, s_lim, sd_lim)

        #plot = pd.generate_state_space_plot(self, self.admissable_region, 1)
        #plot.show()
   
        
        trajectory, intersection_point = pdc.simple_time_optimal_controller(self, (0,0), (1,0), self.bounds)
        
        if plot_trajectory==True:
            pd.generate_control_algorithm_plot(self, self.admissable_region, trajectory, intersection_point, 1, 1)
            
        
        return trajectory
            




    
