from robot_models import two_dof_planar_robot
from functions.example_controller import path_dynamics_controller
import numpy as np
import matplotlib.pyplot as plt
import my_math as mm


def run_lipschitz_region_calculations(manipulator):
    """
    Parameters
    ----------
    robot : parameters to make a robot model
    parameters : simulation parameters
        

    Returns
    -------
    bool for if it is lipschitz, possibly something to plot list of gradients to plot
    Bool

    """
    manipulator.check_if_model_lipschitz()