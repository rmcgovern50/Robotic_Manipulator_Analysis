"""
Created on Tue Feb 11 16:49:47 2020

@author: Ryan McGovern

This module will contain some functions for visualisng stuff with python
these 
"""

import matplotlib.pyplot as plt
from sympy import pprint

def simple_plot(tuple_list, xaxislabel = 'x', yaxislabel = 'y', marker_size=10):
    """
    this function simply takes in a list of tuples and plots them
    Arguments:
        tuple_list [(x1,y1), (x2,y2), ... ,(xn,yn)]
    
    """
    x_val = [x[0] for x in tuple_list]
    y_val = [x[1] for x in tuple_list]
    plt.plot(x_val,y_val,'or',ms=marker_size)
    plt.xlabel(xaxislabel)
    plt.ylabel(yaxislabel)
    plt.show()


def simple_polar_plot(tuple_list, marker_size=1):
    """
    this function simply takes in a list of tuples and plots them in a polar coordinate system thete r
    Arguments:
        tuple_list [(theta1,r1), (theta12,r2), ... ,(theta1n,rn)]
    
    """
    x_val = [x[0] for x in tuple_list]
    y_val = [x[1] for x in tuple_list]

    plt.polar(x_val,y_val,'or',ms=marker_size)
    plt.show()


def equ_print(equation_list):
    #function to pprint a list of symy equations (or expressions)

    for eq in equation_list:
        pprint(eq)
        print("================")