# -*- coding: utf-8 -*-
"""
Created on Thu Jan 15 2021

@author: Ryan McGovern

This script will use my model of a robot to calculate the dynamics
on a path and plot the admissible and inadmissable regions
"""

import sys
sys.path.append('../../../My_modules/my_basic_modules')   #just incase I want to import some modules
sys.path.append('../../../My_modules/my_control_modules') #just incase I want to import some modules
sys.path.append('../../../Robotic_manipulator_control') #just incase I want to import some modules

import math as m
import matlab.engine
import numpy as np

def test_matlab_link_sum(a, b):
    result = eng.sum_parameters(a, b)    
    print("basic summation: ",  a, "+", b, "=", result)   

def add_variable_to_matlab_workspace():
    x = 25.0
    eng.workspace['y'] = x
    a = eng.eval('sqrt(y)')
    print(a)

def test_matlab_pass_array():
    v_in = matlab.double([[5, 10, 15],[1, 2 , 3]])
    scale_factor = 1.0
    result = eng.transpose_and_scale(v_in, scale_factor)    
    print("vin =",  v_in)
    print("vout = ", result)
    

"""
UR5 related examples
"""
def test_matlab_link_UR5_home_config():
    result = eng.UR5_homeconfiguration()
    print("UR5 home configuration ",result)   
    print
def test_matlab_link_symbolic_path():
    q_home = eng.UR5_homeconfiguration()
    q_target = matlab.double([m.pi/2, m.pi/2, m.pi/3, 2*m.pi/3, -m.pi/2, -m.pi/3])
    path = eng.path_symbolic(q_home, q_target)
    print("path: ", path)
    
def test_matlab_UR5_object():
    
    UR5= eng.return_UR5_object()
    
    print("UR5 object", UR5)
    
def test_matlab_robot_object():
    robot_label = 'universalUR5'
    robot = eng.return_robot_object(robot_label)
    print("robot object", robot)

def test_matlab_evaluation_of_path_dynamic_matrices_at_x1_x2_vals():
    x1 = 0.0
    x2 = 1.0
    result = eng.return_evaluated_matrices(x1, x2)
    
    print("M=", result[0], "C=", "g=")
    

"""
stuff to run uncommented below
"""
if __name__ == "__main__": 
    eng = matlab.engine.start_matlab()
    test_matlab_link_sum(2 ,4)
    add_variable_to_matlab_workspace()
    test_matlab_pass_array()
    test_matlab_link_UR5_home_config()
    test_matlab_link_symbolic_path()  
    test_matlab_UR5_object()
    test_matlab_robot_object()
    test_matlab_evaluation_of_path_dynamic_matrices_at_x1_x2_vals()
    eng.quit()
    print("tests complete")
    
    
