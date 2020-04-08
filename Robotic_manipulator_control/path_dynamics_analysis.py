"""
This is a class that will allow path dynamics to be analysed

"""

import my_math as mm# import sub_into_matrix
import matplotlib.pyplot as plt
from sympy import Matrix, diff



class path_dynamics():
    """
    This class will perform the calculations for analysing the admissable 
    region of the robotic manipulators generally
    """
    
    
    def __init__(self, constants_to_sub, s, sd, sdd):
        """
        Arguments:
        constants to sub - list of tuples used to sub in parameters of the robot description
        [(symbol_to_sub1, value_to_sub1), (symbol_to_sub1, value_to_sub1) , ... , (symbol_to_subn, value_to_subn)]
            
        """
        self.constants_to_sub = constants_to_sub
        self.s = s
        self.sd = sd
        self.sdd = sdd

    def calc_qs_matrices(self, qs, qsd, qsdd):
        """
        Arguments, the actuation inputs in terms of s
        return - dynamics matrices in terms of s 
        """
        
        M_explicit = mm.sub_into_matrix(self.M, self.constants_to_sub) 
        C_explicit = mm.sub_into_matrix(self.C, self.constants_to_sub)
        g_explicit = mm.sub_into_matrix(self.g, self.constants_to_sub)
        
        values_to_sub = [(self.q1, list(qs)[0]),(self.q2, list(qs)[1]), \
                         (self.q1d, list(qsd)[0]),(self.q2d, list(qsd)[1]),\
                         (self.q1dd, list(qsdd)[0]),(self.q2dd, list(qsdd)[1])]
        #print(values_to_sub)
        Mqs = mm.sub_into_matrix(M_explicit, values_to_sub)
        Cqs = mm.sub_into_matrix(C_explicit, values_to_sub)
        gqs = mm.sub_into_matrix(g_explicit, values_to_sub)
        
        return Mqs, Cqs, gqs

    def path_parameterisation(self, q):
        """
        Arguments:
            q(s) - the path for each joint parameterised in terms of qs 
                    in the for of a vector of the form:
                    sympy.Matrix[q1(s), q2(s), ..., qn(s)]
            return:
            q(s), dq(s)/dt, d2q(s)/dt2, dq(s)/ds, d2q(s)/ds2
        """
        #define all derivatives needed

        i = 0
        dq_ds = []
        d2q_ds2 = [] 
        
        #differentiate each equation in the matrix
        for fs in q:
            if i == 0:
                dq_ds = [diff(fs, self.s)]
                d2q_ds2 = [diff(dq_ds[0], self.s)]
            else:
                dq_ds.append(diff(fs, self.s))
                d2q_ds2.append(diff(dq_ds[i], self.s))
            i = i + 1

        dqds = Matrix(dq_ds)   
        d2qds2 = Matrix(d2q_ds2)

        qd = Matrix(dq_ds)*self.sd
        qdd = Matrix(dqds)*(self.sdd) + Matrix(d2qds2)*(self.sd)**2
        
        return q, qd, qdd, dqds, d2qds2

    def calc_s_matrices(self, Mqs, Cqs, gqs, dqds,d2sds2):
        """
        Arguments, parameterised M, C and g matrices
        return - M(s), C(S) and G(s) matrices, for 
        M sdd + C sd + g = t(s) form
        """
        
        #print(values_to_sub)
        Ms = Mqs*dqds 
        Cs = Mqs*d2sds2 + (1/(self.sd))*Cqs*dqds
        gs = gqs
        
        return Ms, Cs, gs



    def calc_bounds(self, Ms, Cs, gs):
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
            subtract_part = list(-1*Cs.row(i)*(self.sd)**2 - gs.row(i))[0]
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
        
        boundry_points = []
        
        #move across in the s direction
        while(s <= slims[1]):
            #move up in the s dot dirction
            while(sd <= sdlims[1]):
                #check if a point is admissable
                admissable = self.check_if_addmissable(bounds, s, sd)
                
                #store the point in a list of admissable or inadmissable regions 
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

            #==================can get rid of between equals=====================
            #remember the max value of s dot for each s
            try:
                if boundry_points == [] and ad_region == []:
                    boundry_points = [ad_region[len(ad_region) - 1]]# store this value
                else:
                    boundry_points.append(ad_region[len(ad_region) - 1])
            except:
                print("something went wrong saving the boundry points")
                print("boundry points ->", boundry_points)
                print("ad region ->", ad_region)
            # store this value
            #====================================================================

            s = s + sincrement
            sd = 0
             
        region = 1
        if invert == 0:
            region = ad_region
        elif invert == 1:
            region = non_ad_region
        else:
            raise Exception("invert parameter must equal 0 or 1")
            
        
        return region, boundry_points 


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
        
        s, sd = self.s, self.sd
        
        
        
        L, U = self.calc_upper_lower_bound_values((s_val,sd_val))
        
        if L > U:
            return False
        else:
            return True



    def calc_upper_lower_bound_values(self, point):
        """
        Method to take in a point and evaluate the upper and lower bounds at that point
        Arguments:
            point = (s, sd)
        """
        L = point[0]
        U = point[1]
        bounds = self.bounds
        i = 0

        
        for bound in bounds:
            
            lim_direction_decider = bound[2].subs([(self.s, point[0]), (self.sd, point[1])])
            sub_list = [(self.s, point[0]), (self.sd, point[1])]
            
            #put the bounds the right way round

            if lim_direction_decider > 0:
                L_to_check =  bound[0].subs(sub_list)
                U_to_check = bound[1].subs(sub_list)
              
            elif lim_direction_decider < 0:
                L_to_check =  bound[1].subs(sub_list)
                U_to_check = bound[0].subs(sub_list)
                
            else:
                raise Exception("M(s) cannot be equal to zero - error in calc_upper_lower_bound_values method")
            
            
            
            if i == 0:
                L = L_to_check
                U = U_to_check
            else:
                if L_to_check > L:
                    L = L_to_check
                if U_to_check < U:
                    U = U_to_check
            i = i + 1
            
            
        return L, U


    def evaluate_bounds(self, points_for_evaluation):
        """
        Function simply takes in some points in the s sdot plane and evaluates the bounds
        
        Arguments
            points_for_evaluation = [(s1,sd1),...,(sn,sdn)]
        
        return:
            evaluated_points = [(s1,sd1, L1, U1),...,(sn,sdn, Ln, Un)]
        """
        
        
        evaluated_points = []
        for point in points_for_evaluation:
            s = point[0]
            sd = point[1]
            L, U = self.calc_upper_lower_bound_values(point)
            
            if len(evaluated_points) == 0:
                evaluated_points = [(s, sd, L, U)]
            else:
                evaluated_points.append((s, sd, L, U))
            
            #print(s, sd, L, U)

        evaluated_points 
        return evaluated_points






