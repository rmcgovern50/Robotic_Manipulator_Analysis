# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 14:20:44 2020

@author: Ryan McGovern

This script will use my model of a 2DOF planar robot to calculate the dynamics
on a path and plot the admissable and inadmissable regions
"""


from sympy import symbols, sec, Matrix, latex, pprint
from math import pi

import sys
sys.path.append('../My_modules') #just incase I want to import some modules
from my_visualising import equ_print, simple_plot
from my_math import compare_expression
import robot_models as rm


def test_plot_state_space():

    #Specify robot parameters
    
    Robot_parameters = {
          "m0": 1,#kg
          "l1": 1,#m
          "l2": 1,#m
          "m12": 0.5,#kg
          "q1max": 1,#Nm
          "q1min": -1,#Nm 
          "q2max": 1,#Nm
          "q2min": -1,#Nm
        }
    
    #instatiate class with the physical parameters
    manipulator = rm.two_dof_planar(Robot_parameters)
    
    #form the M(q), C(q,qd) and g(q) matrices
    dynamics = manipulator.dynamics() 
    #pprint(dynamics)
    
    s, sd, sdd = symbols('s sd sdd')
    
    #define parameterisation of path parameters
    """
    q = q(s)
    qd = dq/ds * sd
    qdd = dq/ds * sdd + d2q/ds2 *sd^2
    """
    q = Matrix([[pi/4 - pi/2*s],[pi/4 - pi/2*s]])
    #define all derivatives needed
    dqds = Matrix([[-pi/2],[-pi/2]])
    d2qds2 = Matrix([[0],[0]])

    qd = Matrix([[dqds[0]*sd,[dqds[1]*sd]])
    qdd = Matrix([[dqds[0]*sdd + d2qds2[0]*sd**2],[dqds[1]*sdd + d2qds2[1]*sd**2]])

    #============================================
    #get all the necessary matrices in terms of the parameterised matrices
    Mqs, Cqs, gqs = manipulator.calc_qs_matrices(q, qd, qdd)
    #form M(s), C(s), and g(s) as M(s)*sdd + C(s)*sd**2 + g(s) = t(s)
    Ms, Cs, gs = manipulator.calc_s_matrices(Mqs, Cqs, gqs, dqds, d2qds2, sd)
    #calculate bounds based on the path dynamics
    bounds = manipulator.calc_bounds(Ms, Cs, gs, sd)
    
    #define grid
    s_lim = [0,1,0.1]
    sd_lim = [0,6,0.1]
    #calculate admissable regin
    admissable_region = manipulator.calc_admissable(bounds, s_lim, sd_lim)
    #plot admissable region
    simple_plot(admissable_region, "s", "$\dot{s}$", 1)

    #work still to be completed detailed below
    """
    #next form apply the maximum torque limits 
     
          tmin(s)        <   t(s)   < tmax(s)#torque limits
                             
          so we have some equation of this form that needs to be calculated out
          tmin < m(s)sdd + c(s,sd)sd + g(s) < tmax
          tmin - c(s,sd)sd - g(s) < m(s)sdd < tmax - c(s,sd)sd - g(s)
          
          now taking a row at a time and substituting in the values 
          
          if m(s) > 0
              (tmin - c(s,sd)sd**2 - g(s))/m(s) < sdd < (tmax - c(s,sd)sd**2 - g(s))/m(s)
          elif m(s) < 0
              (tmin - c(s,sd)sd**2 - g(s))/m(s) > sdd > (tmax - c(s,sd)sd**2 - g(s))/m(s)
          
   #next determine the highest lower bound and the lowest upper bound from the list
   
       if lowest upper bound > highest lower bound the state (s, sd) is admissible
       else its in admissable
     
    
    #########################################################
    sdd = manipulator.get_sdd_expressions(parameterised dynamics)
    
    simply loop through the equation list and rearrange each equation as
    sdd = f(s, sd)
       
    return a list of the following form
    [f1(s, sd), f2(s, sd), ..., fn(s, sd)]
    

    
    """
    
    
        
if __name__ == "__main__": 
    test_plot_state_space()
