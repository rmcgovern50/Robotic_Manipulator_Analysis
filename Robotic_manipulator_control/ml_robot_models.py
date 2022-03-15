# -*- coding: utf-8 -*-
"""
This is a file that will contain models for specific robots I wish to study
Each robot, robot specific parts will be coded here
These are all child classes the the path_dynamics_analysis class which helps to 
visualise the state space in a general way
"""

from sympy import symbols, Matrix, sin, cos, acos, asin, atan2, sqrt, pprint, diff

#from path_dynamics_analysis import path_dynamics as pd
#from path_dynamics_control import path_dynamics_controller as pdc
from robot_data_visualisation import two_dof_robot_data_visualisation as rdv

import math as m
import my_math as mm
import numpy as np
import datetime as dt

import my_visualising as mv
import my_sorting as ms

import matplotlib.pyplot as plt
import matlab.engine


class model():
    """
    This class is for the case specific parts of revolute slide robot
    The path_dynamics analysis class will use the information here to
    calculate admissable regions of the state space along a path
    """
    
    def __init__(self, eng, simulation_parameters, current_time):
        """
        Initialise the robot model for manipulation
        """        
        #result = eng.sum_parameters(1.0, 2.0)
        #print(current_time, result)
        self.x1, self.x2, self.u = symbols('x1 x2 u')
        self.robot = simulation_parameters['robot']
        self.path_def = simulation_parameters['path_definition']
        self.x1_lims = simulation_parameters['x1_lim']
        self.x2_lims = simulation_parameters['x2_lim']        
        self.eng = eng

    def joint_space_straight_line_parameterisation(self, q_start, q_end):
        """
        Arguments: matlab arrays
            start = [[q1start, q2start, ... qnstart]]
            end = [[q1end, q2end, ..., qnend ]]
        return:
            qs = [q1(s), q2(s)]
        """
        x1 = self.x1
        #convert to python
        q_0 = Matrix(q_start[0])
        q_1 = Matrix(q_end[0])

        qx1 = q_0 + x1*(q_1-q_0)
        self.qx1 = qx1
        
        return qx1
    
    def example_circular_arc_parameterisation(self, P_start, P_end):
        """
        Arguments: matlab arrays
            start = [[q1start, q2start, ... qnstart]]
            end = [[q1end, q2end, ..., qnend ]]
        return:
            qs = [q1(s), q2(s)]
        """ 
        
        x1 = self.x1

        #convert to python
        P_0 = Matrix([P_start[0], P_start[1]])
        P_1 = Matrix([P_end[0], P_end[1]])

        P_xy_x1 = P_0 + x1*(P_1-P_0)   

        Pz_x1 = sqrt( 0.5 - 2*(P_xy_x1[0]**2) )
        
        #print("z",P_xy_x1[0], P_xy_x1[1], Pz_x1)        
        
        P_x1 = Matrix([P_xy_x1[0], P_xy_x1[1], Pz_x1])
        self.P_x1 = P_x1
            
        return P_x1
    
    def path_parameterisation(self, q):
        """
        Arguments:
            q(s) - the path for each joint parameterised in terms of qs 
                    in the for of a vector of the form:
                    sympy.Matrix[q1(s), q2(s), ..., qn(s)]
            return:
            q(s), dq(s)/dt, d2q(s)/dt2, dq(s)/ds, d2q(s)/ds2
        """
        #define all derivatives needed

        i = 0
        dq_ds = []
        d2q_ds2 = []
        
        #differentiate each equation in the matrix
        for fs in q:
            if i == 0:
                dq_ds = [diff(fs, self.x1)]
                d2q_ds2 = [diff(dq_ds[0], self.x1)]
            else:
                dq_ds.append(diff(fs, self.x1))
                d2q_ds2.append(diff(dq_ds[i], self.x1))
            i = i + 1

        dqds = Matrix(dq_ds)
        d2qds2 = Matrix(d2q_ds2)

        qds = Matrix(dq_ds)*self.x2
        qdds = Matrix(dqds)*(self.u) + Matrix(d2qds2)*(self.x2)**2
        
        return q, qds, qdds, dqds, d2qds2         
        
    def ml_create_circular_arc_waypoints(self):
        """
        Create a set of points that form a circular arc from matlab code
        
        start and end points 
        
        P1 = [ 0.5, 0.5, 0.0]
        P3 = [-0.5, 0.0, 0.0];
        
        path described by  
        y = x + 0.5
        z == (1/2 - 2*x^2)^(1/2)
        
        return-
            wayPoints = [ [xyz_1], [xyz_2], ..., [xyz_n] ]
        """
        
        eng = self.eng
        
        wayPoints = eng.create_circular_arc_waypoints()
        
        return wayPoints
    
    
    def ik_solver(self, wayPoints):
        """
        use Matlab code to perform inverse kinematics for the waypoints
        inputs:
            wayPoints - wayPoints = [ [xyz_1], [xyz_2], ..., [xyz_n] ]
        """
        eng = self.eng
        robot_label = self.robot
        configurations = eng.IK_solver(robot_label, wayPoints)
        
        return configurations

    def run_full_path_dynamics_analysis(self):
        """
        Description-
            This is a method that uses many of the existing methods to produce
            the admissible region plot amoung other information
            that can then be used to design controllers in this space
        
        return:
            region - list of tuples decribing the admissable region
            boundry_points - list of points decribing the boundary
            boundary expressions - list of expressions that when evaluated can get us the upper and lower limits on sdd
            plt - return plot of the admissable region
        """
        eng = self.eng
        x1_lims = self.x1_lims
        x2_lims = self.x2_lims
        path_def = self.path_def
        path_type = path_def[0]        
        #try:
        if path_type == "joint_space_straight_line":
            
            q_start = path_def[1]
            q_end = path_def[2]
            print(q_start, q_end)
            self.joint_space_straight_line_parameterisation(q_start, q_end)
                   
            _, self.qds, self.qdds, self.dqds, self.d2qds2  =\
                self.path_parameterisation(self.qx1)
            
            self.admissible_region, self.boundary_points = self.calc_admissable(x1_lims, x2_lims)
        """
            elif path_type == "circular_arc":
                print("here")
                P0 = (0.5, 0.5, 0.0)
                P1 = (-0.5, 0.0, 0.0)
                Px1 = self.example_circular_arc_parameterisation(P0, P1)
                
                x1 = 0
                x2 = 5
                qx1_evaluated, dqx1_evaluated, ddqx1_evaluated =\
                self.evaluate_path_parameters_workspace_path(Px1, x1, x2)
        except:
            print("could not run simulation")
        """  
    def evaluate_path_parameters_workspace_path(self, Px1, x1, x2):
        """
        evaluate the parameters derivative parameters 
        """
        
        Px1_evaluated = Px1.subs([(self.x1, x1)])
        
        Px1_evaluated = list(Px1_evaluated)
        casted_Px1_evaluated = [float(i) for i in Px1_evaluated]
        
        """
        evaluate q(x1)
        """        
        qx1_evaluated = self.ik_solver(casted_Px1_evaluated)
        
        print("qx1", qx1_evaluated, "at", (x1, x2))

        """
        evaluate dq(x1)/dx1
        """                
        tiny_const = 0.000001
        x1_n = x1 + tiny_const
        
        Px1_n_evaluated = Px1.subs([(self.x1, x1_n)])
        
        Px1_n_evaluated = list(Px1_n_evaluated)
        casted_Px1_n_evaluated = [float(i) for i in Px1_n_evaluated]
        #print("g: ", Px1_evaluated, Px1_n_evaluated)
        qx1_n_evaluated = self.ik_solver(casted_Px1_n_evaluated)
        
        #print("r: ", qx1_evaluated, qx1_n_evaluated)
        qx1_evaluated_matrix = Matrix(qx1_evaluated)
        qx1_n_evaluated_matrix = Matrix(qx1_n_evaluated)
        
        qx1_diff = qx1_n_evaluated_matrix - qx1_evaluated_matrix
        #print("difference in q_1 ", qx1_diff)
        dqx1_evaluated = qx1_diff/tiny_const
        
        #print("derivative->", dqx1_evaluated)
        
        
        #dqx1_evaluated=0
        ddqx1_evaluated=0
        
        return qx1_evaluated, dqx1_evaluated, ddqx1_evaluated


    def evaluate_path_parameters(self, x1, x2):
        """
        evaluate all parameters of the path needed to form limits in matlab
        """
        eng = self.eng
        qx1_evaluated = self.qx1.subs([(self.x1, x1), (self.x2, x2)])
        qx1_evaluated = list(qx1_evaluated)        
        #cast so that sympy is not involved so that the list can be read by matlab
        casted_qx1_evaluated = [float(i) for i in qx1_evaluated]
        #casted_qx1_evaluated2 = [eng.single(i) for i in qx1_evaluated]
        casted_qx1_evaluated = eng.cell(casted_qx1_evaluated)#[casted_qx1_evaluated[0], float(0])
        
        
        dqx1_evaluated = self.dqds.subs([(self.x1, x1), (self.x2, x2)])
        dqx1_evaluated = list(dqx1_evaluated)        
        #cast so that sympy is not involved so that the list can be read by matlab
        casted_dqx1_evaluated = [float(i) for i in dqx1_evaluated]
        casted_dqx1_evaluated = eng.cell(casted_qx1_evaluated)
        
        ddqx1_evaluated = self.d2qds2.subs([(self.x1, x1), (self.x2, x2)])
        ddqx1_evaluated = list(ddqx1_evaluated)        
        #cast so that sympy is not involved so that the list can be read by matlab
        casted_ddqx1_evaluated = [float(i) for i in dqx1_evaluated]
        casted_ddqx1_evaluated = eng.cell(casted_qx1_evaluated)
        
        return casted_qx1_evaluated, casted_dqx1_evaluated, casted_ddqx1_evaluated


    def calc_admissable(self, x1_lims, x2_lims,  invert=0):
        """
        Arguments:
        bounds - list of length n containing tuples of length 3
                tuple element 0 and 1 should contain a bound,
                tuple element 3 contains a value that when evaluated decides
                which bound is the upper and lower
                if element[2] > 0 
                    element[0] is the lower, element[1] is upper 
                if element[2] < 0
                    element[1] is the lower, element[0] is upper
        slims - limits of s in list of form [s minimum, s maximum, increment_size]
        sdlims - limits of sd in list of form [s minimum, s maximum, increment_size]
        invert - return inadmissable region instead of admissable
        
        return:
        return- list of n tuples of length 2 each of which represent a coordinate
        
        """
        eng = self.eng       
        x1 = x1_lims[0]
        x1_inc = x1_lims[2]
        x2 = x2_lims[0]
        x2_inc = x2_lims[2]
        
        #save these variables for other methods
        #self.slims = slims
        #self.sdlims = sdlims
        
        ad_region = []
        non_ad_region = [] 
        
        boundry_points = []
        
        #move across in the s direction
        while(x1 <= x1_lims[1]):
            #move up in the s dot dirction
            print(x1)
            while(x2 <= x2_lims[1]):
                #check if a point is admissable
                
                qx1_evaluated, dqx1_evaluated, ddqx1_evaluated =\
                            self.evaluate_path_parameters(x1, x2)

                admissible = eng.check_if_admissible("universalUR5", \
                                         eng.double(x1), \
                                         eng.double(x2), \
                                         qx1_evaluated, \
                                         dqx1_evaluated, \
                                         ddqx1_evaluated)
                #admissible = eng.check_if_admissible(self.robot, x1, x2)
                #print(admissible, x1, x2)
                #store the point in a list of admissable or inadmissable regions
                if admissible == True: 
                    if len(ad_region) == 0:
                        ad_region = [(x1, x2)]
                    else:
                        ad_region.append((x1, x2))
                else:
                    if len(non_ad_region) == 0:
                        non_ad_region = [(x1, x2)]
                    else:
                        non_ad_region.append((x1, x2))                    
                
                x2 = x2 + x2_inc

            #==================can get rid of between equals=====================
            #remember the max value of s dot for each s
            try:
                if boundry_points == [] and ad_region == []:
                    boundry_points = [ad_region[len(ad_region) - 1]]# store this value
                else:
                    boundry_points.append(ad_region[len(ad_region) - 1])
            except:
                print("something went wrong saving the boundry points")
                print("boundry points ->", boundry_points)
                print("ad region ->", ad_region)
            # store this value
            #====================================================================

            x1 = x1 + x1_inc
            x2 = 0
             
        region = 1
        if invert == 0:
            region = ad_region
        elif invert == 1:
            region = non_ad_region
        else:
            raise Exception("invert parameter must equal 0 or 1")
            
        self.boundary_points = boundry_points
        return region, boundry_points 

                
    def plot_admissible_region(self):
        try:
            current_time =  dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S') #used for naming plots     
            plotter = rdv(current_time)
            plotter.generate_state_space_plot(self.admissible_region,save=False)
        except:
            print("That didn't work, was the simulation run???")
    
    def return_sim_reults(self):
        """
        return data to be pickled as we cannot pickle the object with matlab engine
        """
        
        stuff2pickle = [self.x1, \
                        self.x2, \
                        self.u, \
                        self.robot,\
                        self.path_def,\
                        self.x1_lims,\
                        self.x2_lims,\
                        self.admissible_region,\
                        self.boundary_points,\
                        self.qx1, \
                        self.dqds, \
                        self.d2qds2 ]
        
        return stuff2pickle
    
    def set_sim_results(self, pickledStuff):
        """
        When pickled data is loaded complete the manipulator object  
        """
        
        self.x1 = pickledStuff[0]
        self.x2 = pickledStuff[1]
        self.u = pickledStuff[2]
        self.robot = pickledStuff[3] 
        self.path_def = pickledStuff[4] 
        self.x1_lims = pickledStuff[5] 
        self.x2_lims = pickledStuff[6] 
        self.admissible_region = pickledStuff[7]
        self.boundary_points = pickledStuff[8]
        self.qx1 = pickledStuff[9]
        self.dqds = pickledStuff[10]
        self.d2qds2 = pickledStuff[11]            
            