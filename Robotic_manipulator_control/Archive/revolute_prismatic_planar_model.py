from sympy import symbols, Matrix, sec, tan, sin, cos, Eq, solve, pprint, simplify, lambdify
from math import pi, sqrt
from my_visualising import equ_print
from my_math import compare_expression, sub_into_system_of_equs, sub_into_matrix
from my_sorting import combine_to_tuples

class revolute_prismatic():
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
        

        self.m, self.q1max, self.q1min, self.q2max, self.q2min,\
        = symbols(variables_str)#make symbols for all the varables for symbolic calculations
        
        self.q1, self.q2, self.q1d, self.q2d, self.q1dd, self.q2dd, self.t1, self.t2 \
        = symbols('q1 q2 q1d q2d q1dd q2dd t1, t2')
        
        #list that is of the form that allows easy substitution of robot parameters.
        symbols_to_sub = [self.m, self.q1max, self.q1min, self.q2max, self.q2min,]
        
        values_to_sub = list(Robot_parameters.values())
        
        self.constants_to_sub = combine_to_tuples(symbols_to_sub, values_to_sub)
        
        
    def dynamics(self):
        
        """
        Form the dynamical equations in matrix form, return the result
        """
        #define moments of inertia              
        #define inertia matrix
        M11 = self.m                
        M12 = 0        
        M21 = M12        
        M22 = self.m*self.q1**2
        
        self.M = Matrix([[M11, M12], [M21, M22]])#form inertia matrix
        
        C11 = 0
        C12 = -1*self.m*self.q1*self.q2d
        C21 = 2*self.m*self.q2d        
        C22 = 0
        
        self.C = Matrix([[C11, C12],[C21, C22]])
        
        G1 = self.m*9.8*sin(self.q2)
        G2 = self.m*9.8*self.q1*cos(self.q2)
        
        self.g = Matrix([[G1], [G2]])
        
        self.q = Matrix([[self.q1],[self.q2]])
        
        self.qd = Matrix([[self.q1d], [self.q1d]])

        self.qdd = Matrix([[self.q1dd], [self.q1dd]])
        
        self.inputs = Matrix([[self.t1], [self.t2]])
        
        return self.M, self.C, self.g
                
        
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


    def calc_s_matrices(self, Mqs, Cqs, gqs, dqds,d2sds2, sd):
        """
        Arguments, parameterised M, C and g matrices
        return - M(s), C(S) and G(s) matrices, for 
        M sdd + C sd + g = t(s) form
        """
        
        #print(values_to_sub)
        Ms = Mqs*dqds 
        Cs = Mqs*d2sds2 + (1/sd)*Cqs*dqds
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
            subtract_part = list(-1*Cs.row(i)*sd**2 - gs.row(i))[0]
            divide_part = list(Ms.row(i))[0]
            Ms_part = divide_part
            bound_tuple = ((joint[0] + subtract_part)/divide_part,\
                           (joint[1] + subtract_part)/divide_part,\
                           Ms_part)
            if i == 0:
                #print(Cs.row(i).shape)
                #print((joint[0] + /list(Ms.row(i))[0])
                
                bounds = [bound_tuple]
            else:
                 bounds.append(bound_tuple)    
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
    
