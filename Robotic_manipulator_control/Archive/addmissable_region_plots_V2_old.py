#simple script to evaluate the abmissable regions of a state space from the paper 
#minimum time control of robotic manipulators with geometric path constraints

from sympy import symbols, sec, latex, pprint
from math import pi

import sys
sys.path.append('../My_modules') #just incase I want to import some modules
from my_visualising import equ_print, simple_plot
from my_math import compare_expression
import robot_models as rm


def test_plot_state_space():

    """
    use the robot_models module to import the list of equations that
    parameterise a robot on a path, these equations should be of the form 
    u_i = f(mu, lamda, physical constants)
    The physical constants will be substituted in to begin with but this may be revised 
    to allow for modification
    """
    
    #load in the robot to be analysed
    robot_system = rm.slide_and_rotate_robot()
    
    #xctract the list of parameterised inputs
    #input_equation_list = robot_system.parameterised_equation_list()
    #print("These are the origninal equations")
    #equ_print(input_equation_list)

    
    path_acceleration_list = robot_system.acceleration_equation_list(1)
    #print("These are the origninal equations rearranged in terms of mu dot")
    #equ_print(path_acceleration_list)
    
    
    upper_lim, lower_lim = robot_system.upper_lower_lim_equations(path_acceleration_list)
    
    #print("upper limits are")
    #equ_print(upper_lim)
    
    #print("lower limits are")
    #equ_print(lower_lim)
    
    
    val = robot_system.admissible_finder(0.2, 0.3, upper_lim, lower_lim)
    print(val)
    
    #print(admissable, d)
    
    
    
    
    
    

    #obtain upper and lower bounds

    #evaluate each upper bound for each joint and choose the lowest
    
    #evaluate each lower bound for each joint and choose the highest
    
    #set the admissable region as lower bound (max) < upper bound (min)
    
    #repeat the steps for each value of lambda
    """
    some psuedo code for how the algorithm works
    
    lam = 0 -> max value
    mu = 0 -> max value
    
    
    the nested loops will evaluate each point in a grid and find if a portion is admissable or not
    
    while(lambda < max value)

        while(mu < max value)
        
            
            f(mu lambda) = robot_system.admissible_function_finder(mu lambda)
            
            f(mu lambda) will be an expression found by the following means:
                ->subtract the max lower limit from the min upper limit 
                ->the resulting admissible region will be f(mu lambda) >= 0
                ->so a method must simply return the correct f(mu lambda) to be evaluated
                ->the data needed by function finder method is mu and lambda alone
                ->the function finder show evaluate the limits and take any other stuff into account
            
            if f(mu lambda) >= 0:
                the point is an admissable state (save to list)
            else:
                the point is inadmissable (save to list)
            
 
            mu = mu + mu_incr
        
        
        
        lambda = lambda + lambda_incr
        mu = 0
    plot(addmissble states, inadmissable states) (plot the states in two different colours)
    
        
    #robot_system.admissible_finder(0.3, 0.2, path_acceleration_list)
    #print(admissable, d)
    """
    
    """
    lam_val = 0
    max_lam = pi/4
    lam_inc = 0.1
    
    mu_val = 0
    max_mu = 3
    mu_inc = 0.1
    
    admissable_list = []
    inadmissable_list = [] 
    complete_list = []
    
    while( lam_val < max_lam):
    
        while(mu_val < max_mu):
            #print(mu_val, lam_val)
            admissable=  robot_system.admissible_finder(lam_val, mu_val, upper_lim, lower_lim)
            
            if admissable:
                #save admissable and inadmissable points seperately
                if len(admissable_list) == 0:
                    admissable_list = [(lam_val, mu_val)]
                else:
                    admissable_list.append((lam_val, mu_val))
            else:
                if len(inadmissable_list) == 0:
                    inadmissable_list = [(lam_val, mu_val)]
                else:
                    inadmissable_list.append((lam_val, mu_val))
                
                
                
            mu_val = mu_val + mu_inc    
    
        lam_val = lam_val + lam_inc
        mu_val = 0
        
        
    simple_plot(inadmissable_list, "\u03BB", "\u03BC")
    """
        
if __name__ == "__main__": 
    test_plot_state_space()
