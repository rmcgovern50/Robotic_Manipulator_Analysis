# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 14:20:44 2020

@author: Ryan McGovern

This script will use my model of arobot to calculate the dynamics
on a path and plot the admissable and inadmissable regions
"""

import sys
sys.path.append('../My_modules/my_basic_modules') #just incase I want to import some modules
from my_visualising import simple_plot
from robot_models import revolute_prismatic, cartesian_robot
from sympy import Subs



def test_polar_plot_state_space():
    
    #Specify robot parameters

    #instatiate class with the physical parameters
    manipulator = revolute_prismatic(0.1, [(-1,1), (-1,1)])
      
    
    #form the M(q), C(q,qd) and g(q) matrices
    M, C, g = manipulator.dynamics() 
    #print (M, C ,g)
    
    #describe a path as a function of s
    start_coordinate = [0.5, 3] #enter initial position of end effector [x1, y1]
    end_coordinate = [3, 2] #enter final position of end effector [x2, y2]
    
    qs = manipulator.straight_line_parameterisation(start_coordinate, end_coordinate)
    #print(qs)
    manipulator.plot_end_effector_trajectory(qs, 0.01, 1, 1, 1, 1)
    
    q, qd,qdd, dqds, d2qds2  = manipulator.path_parameterisation(qs)
    #print(q, qd,qdd, dqds, d2qds2)
    #============================================
    #get all the necessary matrices in terms of the parameterised matrices
    Mqs, Cqs, gqs = manipulator.calc_qs_matrices(q, qd, qdd)
    #print(Mqs, Cqs, gqs)
    
    #form M(s), C(s), and g(s) as M(s)*sdd + C(s)*sd**2 + g(s) = t(s)
    Ms, Cs, gs = manipulator.calc_s_matrices(Mqs, Cqs, gqs, dqds, d2qds2)
    #print(Ms, Cs, gs)
    
    #calculate bounds based on the path dynamics
    bounds = manipulator.calc_bounds(Ms, Cs, gs)
    #print(bounds)
    
    #define grid
    s_lim = [0, 1, 0.1]
    sd_lim = [0,10, 0.1]
    
    #calculate admissable region
    admissable_region, boundry_points = manipulator.calc_admissable(bounds, s_lim, sd_lim)
    #print(boundry_points)
    #b = boundry_points[0]
    #print("lower_bound q1 - >", bounds[0][0].subs([(manipulator.s, b[0]),(manipulator.sd, boundry_points[0][1])]))
    
    #print(admissable_region[5])
    #plot admissable region
    simple_plot(admissable_region, "s", "$\dot{s}$", 1)
    

    #work still to be completed detailed below



def test_cartesian_robot_plot():
    
    #Specify robot parameters

    #instatiate class with the physical parameters
    manipulator = cartesian_robot([1, 0.1], [(-1,1), (-1,1)])
      
    
    #form the M(q), C(q,qd) and g(q) matrices
    M, C, g = manipulator.dynamics() 
    #print (M, C ,g)
    
    #describe a path as a function of s
    start_coordinate = [2, 0.5] #enter initial position of end effector [x1, y1]
    end_coordinate = [0.5, 2] #enter final position of end effector [x2, y2]
    
    qs = manipulator.straight_line_parameterisation(start_coordinate, end_coordinate)
    #print(qs)
    manipulator.plot_end_effector_trajectory(qs, 0.01, 1, 1, 1)
    
    q, qd,qdd, dqds, d2qds2  = manipulator.path_parameterisation(qs)
    #print(q, qd,qdd, dqds, d2qds2)
    #============================================
    #get all the necessary matrices in terms of the parameterised matrices
    Mqs, Cqs, gqs = manipulator.calc_qs_matrices(q, qd, qdd)
    #print(Mqs, Cqs, gqs)
    
    #form M(s), C(s), and g(s) as M(s)*sdd + C(s)*sd**2 + g(s) = t(s)
    Ms, Cs, gs = manipulator.calc_s_matrices(Mqs, Cqs, gqs, dqds, d2qds2)
    #print(Ms, Cs, gs)
    
    #calculate bounds based on the path dynamics
    bounds = manipulator.calc_bounds(Ms, Cs, gs)
    #print(bounds)
    
    #define grid
    s_lim = [0, 1, 0.1]
    sd_lim = [0,10, 0.1]
    
    #calculate admissable region
    admissable_region, boundry_points = manipulator.calc_admissable(bounds, s_lim, sd_lim)
    #print(boundry_points)
    #b = boundry_points[0]
    #print("lower_bound q1 - >", bounds[0][0].subs([(manipulator.s, b[0]),(manipulator.sd, boundry_points[0][1])]))
    
    #print(admissable_region[5])
    #plot admissable region
    simple_plot(admissable_region, "s", "$\dot{s}$", 1)
    

    #work still to be completed detailed below

    
    
        
if __name__ == "__main__": 
    test_polar_plot_state_space()
    #test_cartesian_robot_plot() #not vwery useful due to physical makeupb (implemented in like 15 mins which shows the code's good)