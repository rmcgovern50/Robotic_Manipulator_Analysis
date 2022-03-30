import json
import sympy
from sympy import symbols
from math import pi
import os
current_file_path = os.path.dirname(__file__) 
os.chdir(current_file_path)
"""
This script generates a default set of simulation parameters that can be extended if need be
Makes it easy to ensure we run the same simulation twice
"""

def Generate_JSON_parameters():
    
    data = {}
    data['simulation_parameters'] = []
    
    data['simulation_parameters'].append({
        'x1_lim':[0, 0.99, 0.01],\
        'x2_lim':  [0,20, 0.05],\
        'robot': "universalUR5",\
        'path_definition': ["joint_space_straight_line", [(pi/2, pi/2, pi/3, 2*pi/3, -pi/2, -pi/3), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]],\
        'target_set': [(0.9, 4.0), (0.9, 2.0)],\
        'folder_name': "matlab_ur5_default_simulation_1/",\
        'additional_upper_constraint': "x2 - (4*sin(10*x1+5) - 2*sin(18*x1*x1*x1) + 10)",\
        'additional_lower_constraint': "x2 - (4*sin(10*x1+5) - 2*sin(18*x1*x1*x1))",\
        'simulation_label': "universalUR5 sit one"
        })
    data['simulation_parameters'].append({
        'x1_lim':[0, 0.99, 0.2],\
        'x2_lim':  [0,20, 2.5],\
        'robot': "universalUR5",\
        'path_definition': ["joint_space_straight_line", [(pi/2, pi/2, pi/3, 2*pi/3, -pi/2, -pi/3), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]],\
        'target_set': [(0.9, 4.0), (0.9, 2.0)],\
        'folder_name': "matlab_ur5_default_simulation_2/",\
        'additional_upper_constraint': "N/A",\
        'additional_lower_constraint': "N/A",\
        'simulation_label': "universalUR5 sit two"
        })
    data['simulation_parameters'].append({
        'x1_lim':[0, 0.99, 0.01],\
        'x2_lim':  [0,20, 0.05],\
        'robot': "universalUR5",\
        'path_definition': ["joint_space_straight_line", [(pi/2, pi/2, pi/3, 2*pi/3, -pi/2, -pi/3), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]],\
        'target_set': [(0.9, 4.0), (0.9, 2.0)],\
        'folder_name': "matlab_ur5_default_simulation_3/",\
        'additional_upper_constraint': "N/A",\
        'additional_lower_constraint': "N/A",\
        'simulation_label': "universalUR5 sit three"
        })

    with open('default_matlab_simulation_parameters.txt', 'w') as outfile:
        json.dump(data, outfile)

if __name__ == '__main__':
  
    Generate_JSON_parameters()
    with open('default_matlab_simulation_parameters.txt') as json_file:
        data = json.load(json_file)
        sim1= data['simulation_parameters'][2]
        print(sim1['target_set'])

    #Generate_additional_constraints()
