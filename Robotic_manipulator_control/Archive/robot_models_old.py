from sympy import symbols, Matrix, sec, tan, sin, cos, Eq, solve, pprint, simplify, lambdify
from math import pi, sqrt
from my_visualising import equ_print
from my_math import compare_expression, sub_into_system_of_equs, sub_into_matrix
from my_sorting import combine_to_tuples

class two_dof_planar():
    """
    This class is for the 2DOF n   
    """
    
    def __init__(self, Robot_parameters):
        
        self.param = Robot_parameters
        self.joint_limits = [(self.param["q1min"], self.param["q1max"]),\
                             (self.param["q2min"], self.param["q2max"])]

        
        variables = Robot_parameters.keys() #extract variable names from dictionary
        variables = list(variables)
        variables_str = ' '.join(variables)#make string of varable names seperated by a space
        

        self.m0, self.l1, self.l2, self.m12, self.q1max, self.q1min, self.q2max, self.q2min,\
        = symbols(variables_str)#make symbols for all the varables for symbolic calculations
        
        self.q1, self.q2, self.q1d, self.q2d, self.q1dd, self.q2dd, self.Iz1, self.Iz2 , self.t1, self.t2 \
        = symbols('q1 q2 q1d q2d q1dd q2dd Iz1 Iz2 t1, t2')
        
        #list that is of the form that allows easy substitution of robot parameters.
        symbols_to_sub = [self.m0, self.l1, self.l2, self.m12, self.q1max, self.q1min, self.q2max, self.q2min]
        
        values_to_sub = list(Robot_parameters.values())
        
        self.constants_to_sub = combine_to_tuples(symbols_to_sub, values_to_sub)
        
        
    def dynamics(self):
        
        """
        Form the dynamical equations in matrix form, return the result
        """
        #define moments of inertia
        self.Iz1 = self.m12*self.l1**2 \
            + self.m0*((self.l1*cos(self.q1) + self.l2*cos(self.q1 + self.q2))**2 \
            + self.l1*(sin(self.q1) + self.l2*sin(self.q1 + self.q2))**2)
            
        self.Iz2 = self.m0*self.l1**2
        
        
        #define inertia matrix
        M11 = 2*self.m0*self.l1*self.l2*cos(self.q2) +self.Iz1 +self.Iz2 \
                +self.m12*self.l1**2 + self.m0*(self.l1**2 + self.l2**2)
                
        M12 = self.m0*self.l1*self.l2*cos(self.q2) + self.Iz2 + self.m0*self.l2**2
        
        M21 = M12
        
        M22 = self.Iz2 + self.m0*self.l2**2
        
        self.M = Matrix([[M11, M12], [M21, M22]])#form inertia matrix
        
        C11 = -self.m0*self.l1*self.l2*self.q2d*sin(self.q2)
        
        C12 = -self.m0*self.l1*self.l2*(self.q1d +self.q2d)*sin(self.q2)
        
        C21 = -self.m0*self.l1*self.l2*self.q1d*sin(self.q2)
        
        C22 = 0
        
        self.C = Matrix([[C11, C12],[C21, C22]])
        
        G1 = (self.m12 + self.m0)*self.l1*cos(self.q1) + self.m0*self.l2*cos(self.q1 + self.q2)
        G2 = self.m0*self.l2*cos(self.q1 + self.q2)
        
        self.g = Matrix([[G1], [G2]])
        
        self.q = Matrix([[self.q1],[self.q2]])
        
        self.qd = Matrix([[self.q1d], [self.q1d]])

        self.qdd = Matrix([[self.q1dd], [self.q1dd]])
        
        self.inputs = Matrix([[self.t1], [self.t2]])
        

        u = self.M*self.qdd + self.C*self.qd + self.g
        #print(u)
        dynamics = Eq(self.inputs, u)
        
        return dynamics
                
        
    def parametrised_dynamics(self, dynamics, qs, qsd, qsdd):
        """
        Method to take in the matrix representation of the dynamics along with the 
        desired parameterisation q(s) in the form of an nx1 vector
        split the result into a list of all inputs parameterised appropriately
        """
        #print(len(qs))
        #print(qs)
        #L2.subs([(self.lam, lam), (self.mu, mu)])
        
        qs1 = list(qs.row(0))[0]
        qs2 = list(qs.row(1))[0]
        
        qs1d = list(qsd.row(0))[0]
        qs2d = list(qsd.row(1))[0]

        qs1dd = list(qsdd.row(0))[0]
        qs2dd = list(qsdd.row(1))[0]
        
        
        sub_list = [(self.q1, qs1), (self.q2, qs2), \
                    (self.q1d, qs1d), (self.q2d, qs2d), \
                    (self.q1dd,qs1dd), (self.q2dd, qs2dd)]
        
        #print(qs1d)
        
        if len(dynamics.lhs) == len(qs):
            i = 0
            ts = [] #list of paramaterised torques 
            #pprint(dynamics.rhs.row(0))
            
            
            for ex in dynamics.rhs:
            #cycle through each equation and substiute in q, qd, qdd                
                if i == 0:
                    ts = [Eq(list(self.inputs.row(i))[0], ex.subs(sub_list))]
                    
                else:
                    ts.append(Eq(list(self.inputs.row(i))[0], ex.subs(sub_list)))
                i = i + 1
            
            #print("parameterised outputs", ts)
            return ts 
            
        else:
            raise Exception("Impossible to parameterise the dynamics in this way, check the dimensions")
            return 1
        

    def calc_qs_matrices(self, qs, qsd, qsdd):
        """
        Arguments, the actuation inputs in terms of s
        return - dynamics matrices in terms of s 
        """
        
        M_explicit = sub_into_matrix(self.M, self.constants_to_sub) 
        C_explicit = sub_into_matrix(self.C, self.constants_to_sub)
        g_explicit = sub_into_matrix(self.g, self.constants_to_sub)
        
        values_to_sub = [(self.q1, list(qs)[0]),(self.q2, list(qs)[1]), \
                         (self.q1d, list(qsd)[0]),(self.q2d, list(qsd)[1]),\
                         (self.q1dd, list(qsdd)[0]),(self.q2dd, list(qsdd)[1])]
        #print(values_to_sub)
        Mqs = sub_into_matrix(M_explicit, values_to_sub)
        Cqs = sub_into_matrix(C_explicit, values_to_sub)
        gqs = sub_into_matrix(g_explicit, values_to_sub)
        
        return Mqs, Cqs, gqs


    def calc_s_matrices(self, Mqs, Cqs, gqs, dqds,d2sds2):
        """
        Arguments, parameterised M, C and g matrices
        return - M(s), C(S) and G(s) matrices, for 
        M sdd + C sd + g = t(s) form
        """
               
        #print(values_to_sub)
        Ms = Mqs*dqds 
        Cs = Mqs*d2sds2 + Cqs*dqds
        gs = gqs
        
        return Ms, Cs, gs



    def calc_bounds(self, Ms, Cs, gs, sd):
        """
        Arguments: matrices for dynamics on path and the symbol sd
        return:list of bounds as tuples of form
        [(B01,B02, Ms0),(B21,B22, Ms1),(B31,B32, Ms2),...(Bn1, Bn2, Msn)]
        The upper and lower bounds are not descided
        
        Bn1 = (tn_min - C(row n) - G(row n))/M(row n)
        Bn1 = (tn_max - C(row n) - G(row n))/M(row n)
        The third value of the tuple will decide the order of the bounds for a
        particular value of s later on
        """
        
        JntLims = self.joint_limits
        
        
        i = 0
        bounds = []
        
        #loop through each joint and calculate the bounds, 
        #note the upper and lower are undecided and depend on the evaluation
        #of the third value in the tuple
        for joint in JntLims:
            
            #store the variables in an easier form for calculation
            subtract_part = -1*list(Cs.row(i)*sd**2 - gs.row(i))[0]
            divide_part = list(Ms.row(i))[0]
            Ms_part = divide_part
            
            if i == 0:
                #print(Cs.row(i).shape)
                #print((joint[0] + /list(Ms.row(i))[0])
                
                bounds = [((joint[0] + subtract_part)/divide_part,\
                           (joint[1] + subtract_part)/divide_part,\
                           Ms_part)]
            else:
                 bounds.append(((joint[0] + subtract_part)/divide_part,\
                           (joint[1] + subtract_part)/divide_part,\
                           Ms_part))    
            i = i + 1
        

        
        return bounds


    def calc_admissable(self, bounds, slims, sdlims,  invert=0):
        """
        Arguments:
        bounds - list of length n containing tuples of length 3
                tuple element 0 and 1 should contain a bound,
                tuple element 3 contains a value that when evaluated decides
                which bound is the upper and lower
                if element[2] > 0 
                    element[0] is the lower, element[1] is upper 
                if element[2] < 0
                    element[1] is the lower, element[0] is upper
        slims - limits of s in list of form [s minimum, s maximum, increment_size]
        sdlims - limits of sd in list of form [s minimum, s maximum, increment_size]
        invert - return inadmissable region instead of admissable
        
        return:
        return- list of n tuples of length 2 each of which represent a coordinate
        
        """
        
        s = slims[0]
        sincrement = slims[2]
        sd = slims[0]
        sdincrement = sdlims[2]
         
        ad_region = []
        non_ad_region = [] 
        
        while(s < slims[1]):
            
            while(sd < sdlims[1]):
                
                admissable = self.check_if_addmissable(bounds, s, sd)
                
                if admissable == True: 
                    if len(ad_region) == 0:
                        ad_region = [(s, sd)]
                    else:
                        ad_region.append((s, sd))
                else:
                    if len(non_ad_region) == 0:
                        non_ad_region = [(s, sd)]
                    else:
                        non_ad_region.append((s, sd))                    
                    
                sd = sd + sdincrement
                 
            s = s + sincrement
            sd = 0
             
        region = 1
        
        if invert == 0:
            region = ad_region
        elif invert == 1:
            region = non_ad_region
        return region


    def check_if_addmissable(self, bounds, s_val, sd_val):
        """
        Arguments:
            Bounds -A list of length n of tuples length 3 
                    a tuple of length 3 elements 0, and 1 contain bounds in terms of s and sd
                    element 3 contains a value that when evaluated decideds the upper and lower bound
                    
        return:
            true - point is admissable
            false -point in inadmissable
        
        Description:
        This method will take in the list of bounds evaluate the values of all
        lower and upper bounds, work out the min upper bound and max lower bound
        If the max lower > min upper bound the function returns true, else it 
        returns false
        """
        
        s, sd = symbols('s sd')
        
        ordered_bounds = [] #variable to stor the ordered bounds [(lower, upper)]
        
        for bound in bounds:


            lim_direction_decider = bound[2]                            
            lim_direction_decider = lim_direction_decider.subs([(s, s_val), (sd, sd_val)])
            #print(lim_direction_decider)
            leftB = bound[0].subs([(s, s_val), (sd, sd_val)])
            rightB = bound[1].subs([(s, s_val), (sd, sd_val)])
            #if this value is -ve simply swap order fo b
            
            #less than zero flips hhe order
            if lim_direction_decider < 0:
                if len(ordered_bounds) == 0:
                    ordered_bounds = [(rightB, leftB)]
                else:
                    ordered_bounds.append((rightB, leftB))
            #greater than preserves the order    
            elif lim_direction_decider > 0:
                if len(ordered_bounds) == 0:
                    ordered_bounds = [(leftB, rightB)]
                else:
                    ordered_bounds.append((leftB, rightB))
            #equal zero throws error as it doesn't make sense
            else:
                raise Exception("Problem with M(s), this value cannot be zero")        
            
        
        
        #varables to store max and min
        max_lower = 0
        min_upper = 0
        i = 0
        #loop through each bound and find the min upper and max lower bounds
        for bound in ordered_bounds:
            #store the first element
            if i == 0:
                min_upper = bound[1]
                max_lower = bound[0]
            else:
                #if a new min upper is found replace min_upper
                if min_upper > bound[1]:
                    min_upper = bound[1]
                    
                if max_lower < bound[0]:
                    max_lower = bound[0]
            i = i + 1
            
        #print(ordered_bounds)
        #print(max_lower, min_upper)
        
        if max_lower < min_upper:
            #print("admissable")
            return True
        else:
            #print("inadmissable")
            return False
        

    def sub_constants(self, dynamics):
        """
        This function will take in a list of parameterised dynamical equations
        and sub in the constant terms to get a list of equations for the inputs
        in terms of s and its derivatives
        """
        
        dynamics= sub_into_system_of_equs(dynamics, self.constants_to_sub)
        
        return dynamics
 





       
    
    
class slide_and_rotate_robot():
    """
    This class is for the slide and rotate robot from the 1985 paper
    Minimum-Time Control of Robotic Manipulators with Geometric Path Constraints
    Issues exist in code or paper
    """
    def __init__(self):
        
        self.Robot_parameters = {
          "J_theta": 0.001,
          "M_r": 4,
          "L_r": 2,
          "J_p": 10**(-5),
          "M_p": 1,
          "L_p": 0.1,
          "u_r_max": 1,
          "u_theta_max": 1,
          "k_r (low friction)":  0,
          "k_r (high friction)": 15,
          "k_theta": 0,
          "u_r_min": -1,
          "u_theta_min": -1
        }
        self.calculate_implicit_constants()
        
        
        self.K, self.M_t, self.lam, self.J_t, self.k_r, self.k_t, \
        self.u_r, self.u_t, self.mu, self.mu_d,\
        self.u_rmax, self.u_tmax, self.u_rmin, self.u_tmin  = \
        symbols(' K M_t lambda J_t k_r k_theta u_r u_theta mu, mu_dot u_r_max u_t_max u_r_min u_t_min')
        
        
        #define list of constant things to sub into expressions
        self.param_list = [(self.J_t,    self.J_t_val),\
                  (self.K,               self.K_val),\
                  (self.M_t,             self.Mt_val),\
                  (self.k_r,             self.Robot_parameters["k_r (low friction)"]),\
                  (self.k_t,         self.Robot_parameters["k_theta"]), \
                  (self.u_rmax,         self.Robot_parameters["u_r_max"]), \
                  (self.u_rmin,         self.Robot_parameters["u_r_min"]), \
                  (self.u_tmax,         self.Robot_parameters["u_theta_max"]), \
                  (self.u_tmin,         self.Robot_parameters["u_theta_min"]), ]
        
        
        
    def calculate_implicit_constants(self):
        """
        method to calculate some constants used for the dynamics
        """
        self.J_t_val = self.Robot_parameters["J_theta"] + self.Robot_parameters["J_p"] + \
        self.Robot_parameters["M_r"]*(self.Robot_parameters["L_p"]**2 + self.Robot_parameters["L_p"]*self.Robot_parameters["L_r"]\
        +(self.Robot_parameters["L_r"]**2)/3)
        self.Mt_val = self.Robot_parameters["M_r"] + self.Robot_parameters["M_p"]
        self.K_val = self.Robot_parameters["M_r"]*(self.Robot_parameters["L_r"] + 2*self.Robot_parameters["L_p"])
        
        #print("The value of Jt is: ", self.J_t_val)
        #print("The value of Mt is: ", self.Mt_val)
        #print("The value of K is: ", self.K_val)


    def sym_parameterised_equation_list(self):
        """
        method to derive the purely symbolic representation of the system
        all physical constants as variables not evaluated
        """
              
        #create sympy equations fro the paramaterised system
               
        expression_1  = (-self.J_t - self.K*sec(pi/4 - self.lam) + self.M_t*sec(pi/4 - self.lam)**2)*self.mu_d - \
              self.k_t*self.mu  + \
              self.mu**2*(2*self.M_t*sec(pi/4 - self.lam) - self.K)*sec(pi/4 - self.lam)*tan(pi/4 - self.lam)
        equ_1 = Eq(self.u_t,expression_1)
        
        
        
        expression_2 = (-self.M_t*sec(pi/4 - self.lam)*tan(pi/4 - self.lam)*self.mu_d -\
                         self.k_r*sec(pi/4 - self.lam)*tan(pi/4 - self.lam)*self.mu +\
                         (self.M_t*sec(pi/4 - self.lam)*(sec(pi/4 - self.lam)**2 + tan(pi/4 - self.lam)**2) +\
                          self.K/2 - self.M_t*sec(pi/4 - self.lam))*self.mu**2)
        equ_2 = Eq(self.u_r,expression_2)

        
        equation_list = [equ_1 , equ_2]
        return equation_list


    def parameterised_equation_list(self):
        """
        method to simply substitute in the symbolic variables for the robot
        
        """
        equation_list = self.sym_parameterised_equation_list()
        
        
        #loop through equations and sub in values
        i = 0
        for eq in equation_list:
            equation_list[i] = equation_list[i].subs(self.param_list)
            i = i + 1


        return equation_list

    def acceleration_equation_list(self, select_equation_form = 0):
        "method simply rearranges each equation in a list in terms of mu_dot"
                
        #select to use numeric or symbolic equations
        if select_equation_form == 0:
            equations = self.parameterised_equation_list()
        elif select_equation_form                == 1:
            equations = self.sym_parameterised_equation_list()
        else:
            print("Invalid input parameter, using default value (0)" )
            equations = self.parameterised_equation_list()
        

        
        i = 0
        #loop through and replace the elements
        for eq in equations:
            equations[i] = solve(equations[i], self.mu_d)
            i = i + 1

        return equations



    def upper_lower_lim_equations(self, equations):
        """
        function takes in a list of how the acclerations are described and
        outputs 2 lists, an upper limit list and lower limit       
        """
        #define the shape of the limit lists
        lower_lim = [1, 1]
        upper_lim = [2, 2]
   
        #for the revolute joint
        lower_lim[0] = equations[0][0].subs([(self.u_t, self.u_tmin)])
        upper_lim[0] = equations[0][0].subs([(self.u_t, self.u_tmax)])

        #prismatic lambda < pi/4
        lower_lim[1] = equations[1][0].subs([(self.u_r, self.u_rmin)])
        upper_lim[1] = equations[1][0].subs([(self.u_r, self.u_rmax)])


               
        
        return upper_lim, lower_lim



    def admissible_finder(self, lam, mu, upper_lim, lower_lim):
        """
        This method will simply take in a value for mu and lambda and work out 
        and expression that when evaluated if the answer is greater than zero the state is admissable
        """
        
        
        A, B , C = self.limits_from_paper()
        #pprint(simplify(c1))
        #pprint(simplify(c2))
        
        
        
        c1 = A*(self.mu)**2 + B*self.mu + C
        
        c2 = (-1)*A*(self.mu)**2 + (-1)*B*self.mu + C
        
        
        #c1_val = c1.subs(self.param_list)
        #c2_val = c2.subs(self.param_list)
        
        #c1_val =  c1_val.subs([(self.lam, lam), (self.mu, mu)])
        #c2_val =  c2_val.subs([(self.lam, lam), (self.mu, mu)])
                
        if c1_val >= 0 and c2_val >= 0:
            return True
        else:
            return False
        
        
        
        
        
        
        
        
        """
        # save the expressions involved        
        L1 = lower_lim[0]
        L2 = lower_lim[1]
        U1 = upper_lim[0]
        U2 = upper_lim[1] 
        
        
        Calculate expressons to be tested for admissable region, if 
        L1 < U1 -> exp1 =  U1 - L1 > 0
        L2 < U2 -> exp2 =  U2 - L2 > 0  
        L1 < U2 -> exp3 =  U2 - L1 > 0
        L2 < U1 -> exp4 =  U1 - L2 > 0
        
        #exp1 =  U1 - L1
        #exp2 =  U2 - L2 
        #exp3 =  U2 - L1
        #exp4 =  U1 - L2 
        
        L1 = L1.subs(self.param_list)
        L1 = L1.subs([(self.lam, lam), (self.mu, mu)])
        
        L2 = L2.subs(self.param_list)
        L2 = L2.subs([(self.lam, lam), (self.mu, mu)])
        
        U1 = U1.subs(self.param_list)
        U1 = U1.subs([(self.lam, lam), (self.mu, mu)])
        
        U2 = U2.subs(self.param_list)
        U2 = U2.subs([(self.lam, lam), (self.mu, mu)])
        
        #print(L1)
        #print(L2)
        #print(U1)
        #print(U2)
        L = max([L1, L2])
        U = min([U1, U2])
        #print("L ",L )
        #print("U ",U )
        #print("================" )
        #if all these condistions are true the region is admissable
        
        if L<U:
            return True
        else:
            return False
        
        """

        
        
        
        
        
        
        #va, diff = compare_expression(exp1, exp2)
        #pprint(diff)
    
    
        #simply extract the expressions from the paper

        
        
        #print("c1", c1_val)
        #print("c2", c2_val)
        
        #pprint(simplify(exp3 - c2))
        #pprint(simplify(exp4))
        #c3 =  c3.subs([(self.lam, lam), (self.mu, mu)])
        #c4 =  c4.subs([(self.lam, lam), (self.mu, mu)])
        """
        if c3 > 0 and c4 > 0:
            return True
        else:
            return False
        """

        """
        #pprint(c1_val)
        #print("=====================")
        #pprint(c2_val)
        #return 1
        """
        
        
    def limits_from_paper(self):
        """
        This is a simple test function to generate the exact equations from the paper so 
        that my results can be systematically tested
        self.K, self.M_t, self.lam, self.J_t, self.k_r, self.k_t, self.u_r, self.u_t, self.mu, self.mu_d = \
        symbols(' K M_t lambda J_t k_r k_theta u_r u_theta mu, mu_dot')
        
        """
        # linear conditions
        A = -self.K*self.M_t*(sec(pi/4 - self.lam))**4 \
            + 2*self.M_t*(sec(pi/4 - self.lam))**3 \
            + 3/2*self.K*self.M_t*(sec(pi/4 - self.lam))**2 \
            - (2*self.M_t*self.J_t + ((self.K)**2)/2)*sec(pi/2 - self.lam) \
            + (self.K*self.J_t)/2
            
        
        #pprint(A)
        B = (self.J_t*self.k_r - self.M_t*self.k_t)*sec(pi/4 - self.lam)*tan(pi/4 - self.lam) \
            - self.K*self.k_r*(sec(pi/4 - self.lam))**2*tan(pi/4 - self.lam) \
            + self.M_t*self.k_r*tan(pi/4 - self.lam)*(sec(pi/4 - self.lam))**3
        
        C = self.u_rmax*(self.J_t - self.K*sec(pi/4 - self.lam) + self.M_t*(sec(pi/4 - self.lam))**2) \
            + self.u_tmax*self.M_t*tan(pi/4 - self.lam)*sec(pi/4 - self.lam)



        pcond1 = A*(self.mu)**2 + B*self.mu + C
        
        pcond2 = -A*(self.mu)**2 - B*self.mu + C 

        
        #circular conditions

        pcir1 = 2*(self.mu)**2 + (0 - 10)*self.mu*sin(self.lam)*cos(self.lam) + sqrt(2)*(sin(self.lam) + cos(5))
        pcir2 = -2*(self.mu)**2 + (0 - 10)*self.mu*sin(self.lam)*cos(self.lam) + sqrt(2)*(sin(self.lam) + cos(5))


        #mu dot from paper
        sixteena = (self.u_tmax + ((2*self.M_t*sec(pi/4 - self.lam) - self.K) \
                                  *sec(pi/4 - self.lam)*tan(pi/4 - self.lam))*self.mu**2 \
                                  - self.k_t*self.mu)/(self.J_t - self.K*sec(pi/4 - self.lam) + self.M_t*sec(pi/4 - self.lam)**2)
        

        sixteenb = (-1*self.u_tmax + ((2*self.M_t*sec(pi/4 - self.lam) - self.K) \
                                  *sec(pi/4 - self.lam)*tan(pi/4 - self.lam))*self.mu**2 \
                                  - self.k_t*self.mu)/(self.J_t - self.K*sec(pi/4 - self.lam) + self.M_t*sec(pi/4 - self.lam)**2)


        sixteenc = (self.u_rmax + (2*self.M_t*sec(pi/4 - self.lam)*tan(pi/4 - self.lam)**2 + self.K/2)*self.mu**2 \
                    - self.k_r*self.mu*sec(pi/4 - self.lam)*tan(pi/4 - self.lam)) \
                    /(self.M_t*sec(pi/4 - self.lam)*tan(pi/4 - self.lam))


        sixteend = (-self.u_rmax + (2*self.M_t*sec(pi/4 - self.lam)*tan(pi/4 - self.lam)**2 + self.K/2)*self.mu**2 \
                    - self.k_r*self.mu*sec(pi/4 - self.lam)*tan(pi/4 - self.lam)) \
                    /(self.M_t*sec(pi/4 - self.lam)*tan(pi/4 - self.lam))




        return A, B, C#pcond1, pcond2, pcir1, pcir2#, sixteena, sixteenb, sixteenc, sixteend





