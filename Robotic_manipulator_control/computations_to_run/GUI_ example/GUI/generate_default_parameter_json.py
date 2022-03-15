import json
import sympy
from sympy import symbols

"""
This script generates a default set of simulation parameters that can be extended if need be
Makes it easy to ensure we run the same simulation twice
"""


def Generate_JSON_parameters():
    
    data = {}
    data['simulation_parameters'] = []
    
    data['simulation_parameters'].append({
        'robot': {'joint_masses': [0.25, 0.25],\
        'link_lengths': [0.2,0.2],\
        'actuator_limits': [(-10,10), (-10,10)]},\
        'x1_lim':[0, 1, 0.01],\
        'x2_lim':  [0,20, 0.1],\
        'path_definition': ["straight line", [(0.34,-0.20),(0.34, 0.20)]],\
        'target_set': [(0.9, 8.5), (0.9, 8.5)],\
        'folder_name': "default_simulation_0/",\
        'additional_upper_constraint': "x2 - (4*sin(10*x1+5) - 2*sin(18*x1*x1*x1) + 10)",\
        'additional_lower_constraint': "N/A"\
        })
    data['simulation_parameters'].append({
        'robot':{'joint_masses': [0.25, 10],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits':  [(-10,10), (-10,10)]},\
        'x1_lim':[0, 1, 0.01],\
        'x2_lim':  [0,20, 0.01],\
        'path_definition': ["straight line", [(0.34,-0.20),(0.34, 0.20)]],\
        'target_set': [(1, 7.45),(1, 7.45)],\
        'folder_name': "default_simulation_1/",\
        'additional_upper_constraint': "x2 - (4*sin(10*x1+5) - 2*sin(18*x1*x1*x1) + 10)",\
        'additional_lower_constraint': "x2 - (4*sin(10*x1+5) - 2*sin(18*x1*x1*x1))"\
        })
    data['simulation_parameters'].append({
        'robot':{'joint_masses': [0.25, 0.25],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits': [(-10,10), (-10,10)]},\
        'x1_lim':[0, 1, 0.01],\
        'x2_lim':  [0,20, 0.01],\
        'path_definition': ["circular arc", [(0.1, 0.30), 0.15]],\
        'target_set': [(1, 8.5), (1, 2.8)],\
        'folder_name': "default_simulation_2/",\
        'additional_upper_constraint': "x2 - (4*sin(10*x1+5) - 2*sin(18*x1*x1*x1) + 10)",\
        'additional_lower_constraint': "N/A"\
        
        })
    
    data['simulation_parameters'].append({
        'robot':{'joint_masses': [0.25, 0.25],\
             'link_lengths': [0.2,0.2],\
             'actuator_limits': [(-10,10), (-10,10)]},\
        'x1_lim':[0, 1, 0.005],\
        'x2_lim':  [0,10, 0.01],\
        'path_definition': ["circular arc", [(0.1, 0.30), 0.15]],\
        'target_set': [(1, 8.5), (1, 4)],\
        'folder_name': "default_simulation_3/",\
        'additional_upper_constraint': "x2 - (4*sin(10*x1+5) - 2*sin(18*x1*x1*x1) + 10)",\
        'additional_lower_constraint': "x2 - (4*sin(10*(x1 - 1)+5) - 2*sin(18*(x1*x1*x1 - 2)) - 2)"\
        
        })
        
        
    with open('default_simulation_parameters.txt', 'w') as outfile:
        json.dump(data, outfile)



if __name__ == '__main__':
  
    Generate_JSON_parameters()
    with open('default_simulation_parameters.txt') as json_file:
        data = json.load(json_file)
        sim1= data['simulation_parameters'][2]
        print(sim1['target_set'])

    #Generate_additional_constraints()
