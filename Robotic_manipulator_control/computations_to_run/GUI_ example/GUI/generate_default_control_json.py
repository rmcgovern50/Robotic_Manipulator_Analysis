import json
import sympy
from sympy import symbols
import os
current_file_path = os.path.dirname(__file__) 
os.chdir(current_file_path)
"""
This file will allow us to generate standardised controller JSON files to setup a controller
Identically over various runs
"""


def Generate_JSON_parameters():
    
    data = {}
    data['control_parameters'] = []
    
    data['control_parameters'].append({
        'stationary_time': True,\
        'controller_label': "default using boundaries",\
        'control_guide_type': "raw",\
        'initial_state': (0, 0.01),\
        'integration_step': 0.05,\
        'upper_guide': "N/A",\
        'lower_guide': "N/A",\
        'folder_name': "control_data/",\
        'file_name': "default using boundaries",\
        })
        
    data['control_parameters'].append({
        'stationary_time': True,\
        'controller_label': "default custom guides",\
        'control_guide_type': "custom",\
        'initial_state': (0, 1),\
        'integration_step': 0.01,\
        'upper_guide': "0.5*x1*sin(60*x1)+4*x1+3.5",\
        'lower_guide': "0.75*cos(12*x1)+2*x1+2",\
        'folder_name': "control_data/",\
        'file_name': "default using custom guides",\
        })
        

    with open('default_control_parameters.txt', 'w') as outfile:
        json.dump(data, outfile)



if __name__ == '__main__':
  
    Generate_JSON_parameters()
    with open('default_control_parameters.txt') as json_file:
        data = json.load(json_file)
        sim1= data['control_parameters'][1]
        print(sim1['controller_label'])

    #Generate_additional_constraints()
