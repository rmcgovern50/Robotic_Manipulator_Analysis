#simple script to evaluate the abmissable regions of a state space from the paper 
#minimum time control of robotic manipulators with geometric path constraints

from sympy import symbols, sec, latex
from math import pi

import sys
sys.path.append('../My_modules') #just incase I want to import some modules
import my_visualising as mv

def test_plot_state_space():

    K, M_t, lam, J_t, k_r, k_theta, u__r_max, u__theta_max, mu = \
    symbols(' K M_t lambda J_t k_r k_theta u__r_max u__theta_max mu')
    
    A = -K*M_t*(sec(pi/4 - lam)**4) + 2*M_t*(sec(pi/4 - lam)**3) + \
    (3/2)*K*M_t*(sec(pi/4 - lam)**2) - (2*M_t*J_t + (K**2)/2)*sec(pi/2 - lam) + \
    K*J_t/2
    
    B = (J_t*k_r - M_t*k_theta)*sec(pi/4 - lam)*tan(pi/4 - lam) -\
    K*k_r*(sec(pi/4 - lam))**2*tan(pi/4 - lam) + \
    M_t*k_r*tan(pi/4 - lam)*(sec(pi/4 - lam))**3
    
    C = u__r_max*(J_t - K*sec(pi/4 - lam) + M_t*(sec(pi/4 - lam))**2) +\
    u__theta_max*M_t*tan(pi/4 - lam)*sec(pi/4 - lam)

    #print("A = ", A)
    #print("B = ", B)
    #print("C = ", C)
    
    Robot_parameters = {
      "J_theta": 10**(-3),
      "M_r": 4,
      "L_r": 2,
      "J_p": 10**(-5),
      "M_p": 1,
      "L_p": 0.1,
      "u_r_max": 1,
      "u_theta_max": 1,
      "k_r (low friction)":  0,
      "k_r (high friction)": 15,
      "k_theta": 0
    }
    
    #calculate parameters needed for substitution
    J_t_val = Robot_parameters["J_theta"] + Robot_parameters["J_p"] + \
    Robot_parameters["M_r"]*(Robot_parameters["L_p"]**2 + Robot_parameters["L_p"]*Robot_parameters["L_r"]\
    +(Robot_parameters["L_r"]**2)/3)
    
    Mt_val = Robot_parameters["M_r"] + Robot_parameters["M_p"]
    
    K_val = Robot_parameters["M_r"]*(Robot_parameters["L_r"] + 2*Robot_parameters["L_p"])
    
    #define list of constant things to sub into expressions
    param_list = [(J_t,             J_t_val),\
                  (K,               K_val),\
                  (M_t,             Mt_val),\
                  (k_r,             Robot_parameters["k_r (low friction)"]),\
                  (k_theta,         Robot_parameters["k_theta"]),\
                  (u__r_max,        Robot_parameters["u_r_max"]),\
                  ( u__theta_max,   Robot_parameters["u_theta_max"])]
    
    
    #form expression, if this is >=0 then it is within the admissable region (for lambda < pi/4)
    expression_1 = A*(mu**2) + B*mu + C

    #substitute in any constant robot physical parameter values
    expression_1 = expression_1.subs(param_list)
    
    #form expression, if this is >=0 then it is within the admissable region (for lambda < pi/4)
    expression_2 = -A*(mu**2) - B*mu + C

    #sub in values
    expression_2 = expression_2.subs(param_list)
    
    #print("lambda < pi over 4 exp" , expression_1)
    #print("lambda >= pi over 4 exp", expression_2)
    
    #loop through and simulate the system
    
    lam_val = 0
    max_lam = pi/4
    lam_inc = 0.01
    
    mu_val = 0
    max_mu = 1
    mu_inc = 0.01
    
    admissable_list = []
    inadmissable_list = [] 
    complete_list = []
    while( lam_val < max_lam):
    
        while(mu_val < max_mu):
            
            if lam_val < pi/4:
               ans = expression_1.subs([(lam, lam_val), (mu, mu_val)])
               #print("got in here", ans)
            else:
               ans = expression_2.subs([(lam, lam_val), (mu, mu_val)])
               #print("got here", ans)
     
            #save admissable and inadmissable points seperately
            if ans >= 0:
                if len(admissable_list) == 0:
                    admissable_list = [(lam_val, mu_val)]
                else:
                    admissable_list.append((lam_val, mu_val))
            else:
                if len(inadmissable_list) == 0:
                    inadmissable_list = [(lam_val, mu_val)]
                else:
                    inadmissable_list.append((lam_val, mu_val))
            
            #produce a big list of all the data
            if len(complete_list) == 0:
                complete_list = [(lam_val, mu_val, ans)]
            else:
                complete_list.append((lam_val, mu_val, ans))
                
            mu_val = mu_val + mu_inc    
    
        lam_val = lam_val + lam_inc
        mu_val = 0
    
    #print("admissable_points", admissable_list)
    #print("admissable_points", inadmissable_list)

    #roughly view the inadmissable region
    mv.simple_plot(admissable_list, "\u03BB", "\u03BC")
    #print(complete_list)
    
if __name__ == "__main__": 
    test_plot_state_space()
