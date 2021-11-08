"""
This is a class that will allow path dynamics to be analysed

"""

import my_math as mm# import sub_into_matrix
import matplotlib.pyplot as plt
from sympy import Matrix, diff, Transpose, pprint, latex
import numpy as np


class path_dynamics():
    """
        This class will perform the calculations for analysing the admissible 
        region of the robotic manipulators generally
    """

    
    def __init__(self, M, C, g, q1, q2, q1d, q2d, q1dd, q2dd, constants_to_sub, s, sd, sdd, qd):
        """
        Arguments:
        constants to sub - list of tuples used to sub in parameters of the robot description
        [(symbol_to_sub1, value_to_sub1), (symbol_to_sub1, value_to_sub1) , ... , (symbol_to_subn, value_to_subn)]
            
        """
        self.constants_to_sub = constants_to_sub
        self.s = s
        self.sd = sd
        self.sdd = sdd
        self.qd = qd
        self.M = M
        self.C = C
        self.g =g
        self.q1 = q1
        self.q2 = q2
        self.q1d = q1d
        self.q2d = q2d
        self.q1dd = q1dd
        self.q2dd = q2dd
        
        
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

    def calc_qs_Jacobian(self, Jacobian, qs, qds, qdds):
        """
        Method that takes in the path parameterisation as well as a jocobian of some point on the
        manipulator and returns an explicit version in terms of s and it's derivatives'
        Arguments:
            qs, qds, qdds
            
        Return:
            Jqs - jacobian in terms of s
        """
        J_explicit = mm.sub_into_matrix(Jacobian, self.constants_to_sub) 
        
        values_to_sub = [(self.q1, list(qs)[0]),(self.q2, list(qs)[1]), \
                         (self.q1d, list(qds)[0]),(self.q2d, list(qds)[1]),\
                         (self.q1dd, list(qdds)[0]),(self.q2dd, list(qdds)[1])]
            
        Jqs = mm.sub_into_matrix(J_explicit, values_to_sub)
        return Jqs

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

        qds = Matrix(dq_ds)*self.sd
        qdds = Matrix(dqds)*(self.sdd) + Matrix(d2qds2)*(self.sd)**2
        
        return q, qds, qdds, dqds, d2qds2

    def calc_s_matrices(self, Mqs, Cqs, gqs, dqds,d2sds2):
        """
        Arguments, parameterised M, C and g matrices
        return - M(s), C(S) and G(s) matrices, for 
        M sdd + C sd + g = t(s) form
        """
        
        #print(values_to_sub)
        self.Ms = Mqs*dqds 
        Cs = Mqs*d2sds2 + (1/(self.sd))*Cqs*dqds
        gs = gqs
        
        return self.Ms, Cs, gs

    
    def get_machine_kinetic_energy_q(self):
       
        kinetic_energy_expression_q = 0.5*Transpose(self.qd)*self.M*self.qd
        kinetic_energy_expression_q = kinetic_energy_expression_q.subs(self.constants_to_sub)
        return kinetic_energy_expression_q
    

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
                #bound for joint 1
                bounds = [bound_tuple]
            else:
                #bounds for each other joint
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
        sd = sdlims[0]
        sdincrement = sdlims[2]
        
        #save these variables for other methods
        self.slims = slims
        self.sdlims = sdlims
        
        ad_region = []
        non_ad_region = [] 
        
        boundry_points = []
        
        #move across in the s direction
        while(s <= slims[1] + sincrement):
            #move up in the s dot dirction
            while(sd <= sdlims[1] + sdincrement):
                #check if a point is admissable
                admissable = self.check_if_addmissable(bounds, s, sd)
                #print(s)
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
            
        self.boundary = boundry_points
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
        
        L, U = self.calc_upper_lower_bound_equations((s_val,sd_val),return_value=True)
        
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
        print("obsolete function please use calc_upper_lower_bound_equations()")
        L = point[0]
        U = point[1]
        bounds = self.bounds
        i = 0

        #print(L, U)
        for bound in bounds:
            #print(point)
            lim_direction_decider = bound[2].subs([(self.s, point[0]), (self.sd, point[1])])
            sub_list = [(self.s, point[0]), (self.sd, point[1])]
            
            #put the bounds the right way round
            try:
                
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
            except:
                print("weird direction decider = ", lim_direction_decider)
                pass
            i = i + 1
            
        return L, U

    def pass_qs_qds_qdds(self, qs, qds, qdds):
        self.qs = qs
        self.qds = qds
        self.qdds = qdds



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
            
 
        return evaluated_points


    def evaluate_potential_collision_energy(self, J, M, n, s_plane_trajectory, return_symbolic_equation=False):
        """
            Funtion which takes in a jacobian, mass matrix and drection of collision
            Arguments:
                J - jacobian of collision point
                M - mass matrix of collision point
                n - direction of collision
                s_plane_trajectory - coordinates to evaluate energy 
                                    [(s1, sd1),...,(sn, sdn)]
                
            returns - the list of potential collision energy values
                        [(s1, sd1, Ek1),...,(sn, sdn, Ekn)]
        """
     
        deltaEk_list = [(0,0,0)]
        i = 0
        
        
        Jqs = self.calc_qs_Jacobian(J,  self.qs, self.qds, self.qdds)

        Jqsn = n.T*Jqs
        
        #calculate 
        mn_temp = Jqsn*M*(Jqsn.T)
        xdotn = Jqsn*self.qds        
        
        deltaEk_symbolic=0.5*mn_temp*xdotn**2
        
        for ssd in s_plane_trajectory:
            mn_explicit = mm.sub_into_matrix(mn_temp, [(self.s, ssd[0])])
            mn = mn_explicit**-1

            xdotn_explicit = mm.sub_into_matrix(xdotn, [(self.s, ssd[0]), (self.sd, ssd[1])])
    
            deltaEk = 0.5*mn*xdotn_explicit**2
            if i == 0:
                deltaEk_list[0] = (ssd[0], ssd[1], deltaEk[0])
                i=1
            else:
                deltaEk_list.append((ssd[0], ssd[1], deltaEk[0]))

        if return_symbolic_equation == False:
            return deltaEk_list
        else:
            pass#functionality not added yet
        

    def convert_admissible_region_boundry_to_polynomial(self, boundary_to_approximate, plot=False):
        """
        simple method to obtain a conservative polynomial

        """
                
        x_val = [x[0] for x in boundary_to_approximate]
        y_val = [x[1] for x in boundary_to_approximate]
        
        x=np.array(x_val)
        y=np.array(y_val)
        #z1 = np.polynomial.Polynomial.fit(x, y, 2)
        
        i = 3
        fit_close_enough = False
        desired_closeness = 0.05
        while i < 1000 and fit_close_enough == False:
            
            z=np.polyfit(x,y, i) #x and y describe function to be approximated, the number is the order of polynomial
            y_calc = np.polyval(z, x)
            
            y_calc=np.array(y_calc)
            
            difference = y - y_calc
            #print(max(difference))
            if max(difference) > desired_closeness: 
                i = i + 1
            else:
                fit_close_enough = True
                
        print(i, fit_close_enough)
        if plot ==True:
            print(z)
            #print(z1, z)
            plt.plot(x, y_calc, color='g')
            plt.plot(x, y, 'or', color='b', ms=1)
    
            plt.xlabel("s")
            plt.ylabel("$\dot{s}$")
            plt.ylim(ymin=0)
            plt.show()
        
        return z, x
    
    def create_energy_limit_projection(self, actuator_constraint_boundary, energy_limit):
        
        new_admissable_boundary = []
        i=0
        #print(actuator_constraint_boundary)
        #loop through the list and save all values needed for plot
        for el in actuator_constraint_boundary:
            #print(el[2])
            if el[2] < energy_limit:
                if i == 0:
                    new_admissable_boundary =  [el[:2]]
                    i=1
                else:
                    new_admissable_boundary.append(el[:2])
                    
        return new_admissable_boundary
    
    def create_boundary_description(self, region):
        #function to move through and isolate the boundary of a given region (assuming the boundary is at the top)
                        
        s = [x[0] for x in region]
        sd = [x[1] for x in region]
        i=0
        #print(region)
        boundary_list = []
        
        #loop through and save the boundary elements
        while i < len(region)-1:
            s_current = s[i]
            s_next = s[i + 1]
            
            if s_current != s_next:
                if len(boundary_list) == 0:
                    boundary_list = [(s[i], sd[i])]
                else:
                    boundary_list.append((s[i], sd[i]))
            i = i + 1
            
        boundary_list.append((s[-1], sd[-1])) #save the final boundary point   
        #print(boundary_list)
        return boundary_list
    
    def get_region_acceleration_bound_equations(self):
        B11= self.bounds[0][0]
        B12 = self.bounds[0][1]
        B1_direction_decider = self.bounds[0][2]
        B21= self.bounds[1][0]
        B22 = self.bounds[1][1]    
        B2_direction_decider = self.bounds[1][2]        
        print("Actuator 1, Bound 1 ", latex(B11))
        print("============================")
        print("Actuator 1, Bound 2 ", latex(B12))
        print("============================")
        print("Actuator 1, direction deciding equation ", latex(B1_direction_decider))                
        print("============================")        
        print("Actuator 2, Bound 1 ", latex(B21))
        print("============================")        
        print("Actuator 2, Bound 2 ", latex(B22))
        print("============================")  
        print("Actuator 2, direction deciding equation ", B2_direction_decider)         
        print("============================")        

    def get_admissible_region_sorted_by_acceleration_bounds(self):
        """
        method that takes the admissible region and splits it based on
        the maximum acceleration and deceleration bounds at different
        points

        Returns
        -------
        new_region : list of lists of tuples
            DESCRIPTION.
            4 lists of tuples containing s sdot coordinates
            list 1 is U1, L1 as bounds
            list 2 is U1, L2 as bounds
            list 3 is U2, L1 as bounds
            list 4 is U2, L2 as bounds
        """
        new_region = [[()],[()],[()],[()]]
        new_region_3D = [[()],[()],[()],[()]]
        #limiting bounds on actuator 1
        B11= self.bounds[0][0]
        B12 = self.bounds[0][1]
        
        #limiting bounds on actuator 2
        B21= self.bounds[1][0]
        B22 = self.bounds[1][1]                
        
        admissible_region = self.admissible_region
            
        umin_index = 0 # index to mark the region label
        lmax_index = 0 # index to mark the region label
        
        
        
        for point in admissible_region:
            #print(point)
            
            variables_to_sub = [(self.s, point[0]), (self.sd, point[1])]
            
            #calculate actuator 1 limits
            B11_val = B11.subs(variables_to_sub)
            B12_val = B12.subs(variables_to_sub)
            
            #define the upper and lower bounds
            if B11_val > B12_val:
                U1 = B11_val
                L1 = B12_val
            else:
                U1 = B12_val
                L1 = B11_val                
            
            #calculate actuator 2 limits
            B21_val = B21.subs(variables_to_sub)
            B22_val = B22.subs(variables_to_sub)
            
            #define the upper and lower bounds
            if B11_val > B12_val:
                U2 = B21_val
                L2 = B22_val
            else:
                U2 = B22_val
                L2 = B21_val  
            
            if U1 > U2:
                Umin = U2
                umin_index = 2
            else:
                Umin = U1
                umin_index = 1                
            
            if L1 > L2:
                Lmax = L1
                lmax_index = 1
            else:
                Lmax = L2
                lmax_index = 2

            
            if umin_index == 1 and lmax_index == 1:
                #print("x1")
                new_region[0].append(point)
                new_region_3D[0].append((point[0],point[1], Lmax, Umin))
            elif  umin_index == 1 and lmax_index == 2:
                #print("x2")
                new_region[1].append(point)
                new_region_3D[1].append((point[0],point[1], Lmax, Umin))
            elif  umin_index == 2 and lmax_index == 1:
                #print("x3")
                new_region[2].append(point)
                new_region_3D[2].append((point[0],point[1], Lmax, Umin))
            elif  umin_index == 2 and lmax_index == 2:
                #print("x4")
                new_region[3].append(point)
                new_region_3D[3].append((point[0],point[1], Lmax, Umin))
                    
            umin_index = 0 # reset
            lmax_index = 0 # reset         
            
        new_region[0].pop(0)
        new_region[1].pop(0)
        new_region[2].pop(0)
        new_region[3].pop(0)

        new_region_3D[0].pop(0)
        new_region_3D[1].pop(0)
        new_region_3D[2].pop(0)
        new_region_3D[3].pop(0)

        return new_region, new_region_3D
    
    
    def check_if_list_is_made_of_multiple_sets(self, region_to_test, plot=True):
        """
        

        Parameters
        ----------
        region_to_test : list of points 
            DESCRIPTION. list of points making up some region
        plot : bool, optional
            DESCRIPTION. If true produce a plot

        Returns
        -------
        boundary_points : list of points
            DESCRIPTION. list  of points around the boundary of the input region

        """
        slims = self.slims
        sdlims = self.sdlims
        
        s_min = slims[0]
        s_increment = slims[2]
        s_max = slims[1]
        
        sd_min = sdlims[0]
        sd_increment = sdlims[2]        
        sd_max = sdlims[1]
        
        i = 0
        B = 0
        #print(len(region_to_test))
        boundary_points = [()]
        for suspect_boundary_point in region_to_test:
            
            s = suspect_boundary_point[0]
            sd = suspect_boundary_point[1]
            
            #define the points that exist in the non boundary point case
            point_right  = (s + s_increment , sd)
            point_left =   (s - s_increment , sd)
            point_above  = (s               , sd + sd_increment)
            point_below =  (s               , sd - sd_increment)            

            sr = round(point_right[0], 4)
            sdr = round(point_right[1], 4)

            sl = round(point_left[0], 4)
            sdl = round(point_left[1], 4)
            
            sa = round(point_above[0], 4)
            sda = round(point_above[1], 4)

            sb = round(point_below[0], 4)
            sdb = round(point_below[1], 4)
            
            #set flags
            l_found = False
            r_found = False
            a_found = False
            b_found = False
            
            #loop through the region and attempt to find all 
            #4 points
            points_searched=0
            for point in region_to_test:
                sp = round(point[0], 4)
                sdp = round(point[1], 4)
                
                if sp == sr and sdp == sdr:
                    r_found = True
                elif sp == sl and sdp == sdl:
                    l_found = True
                elif sp == sa and sdp == sda:
                    a_found = True
                elif sp == sb and sdp == sdb:
                    b_found = True
                #print("sides found",l_found, r_found , a_found, b_found )
                points_searched = points_searched + 1 
                if l_found and r_found and a_found and b_found:
                    #print("boundary point is definitely not ", suspect_boundary_point)
                    break
                
            if points_searched < len(region_to_test):
                i = i + 1 #print("not a boundary point")
            else:
                boundary_points.append(suspect_boundary_point)
                B = B + 1
                #print("is a boundary point")
        
                
        #print(i, B)
        #print(boundary_points)
        boundary_points.pop(0)
        if plot:
            x = [x[0] for x in boundary_points]
            y = [x[1] for x in boundary_points]
            
            plt.plot(x, y, 'or', color='y', ms=1)
            plt.xlabel("s")
            plt.ylabel("$\dot{s}$")
            plt.ylim(ymin=0)
            plt.show()
        return boundary_points

    def sort_boundary(self, boundary_points, plot=False):
        

        slims = self.slims
        sdlims = self.sdlims

        #define grid increments
        s_increment = slims[2]
        sd_increment = sdlims[2]        
        
        #define grid ratio sd/s
        ratio = (sd_increment/s_increment)

        largest_gap_between_adjacent_points = np.sqrt(np.square(ratio*(s_increment)) + np.square(sd_increment))*1.1#add 10% incase rounding
        #each point on the boundary will have two closest points
        #choose a random point and find the closest point
        #add the closest point to the list and remove it from the searchable list
        sorted_list = [(boundary_points[0])]
        boundary_points.pop(0)
        
        weird_points = [(0,0)]
        i=0

        while len(boundary_points) > 0:
            
            #form lists of the s and sd coordinates for the remaining boundary    
            s = [x[0] for x in boundary_points]
            sd = [x[1] for x in boundary_points]
            
            #convert to np arrays for speed
            s_boundary = np.array(s)
            sd_boundary = np.array(sd)
           
            #take the latest element from the sorted list and form a vector to
            #do vector compuations with all other points on the boundary
            s_current = sorted_list[i][0]
            sd_current = sorted_list[i][1]
            
            s_current_vec = s_current*np.ones(len(boundary_points))
            sd_current_vec = sd_current*np.ones(len(boundary_points))
            
            #calculate the distance between the latest sorted element and all other points
            #record the min distance and the index of the value

            distances = np.sqrt(np.square(ratio*(s_current_vec - s_boundary)) + np.square(sd_current_vec - sd_boundary))
            min_distance = np.min(distances)
            min_distance_index = np.argmin(distances)
            #print(min_distance)
            #add the closest point to the sorted list and remove the point from the boundary list or future searches

            
            if min_distance > largest_gap_between_adjacent_points:
                #need to better isolate how to deal with these situations
                #these points are often generated due to the presence of multiple regions
                #or there being a thin part of one region leading to multiple choices
                #print("large minimum distance", min_distance)
                weird_points.append(boundary_points[min_distance_index])
                i = i - 1
                #print()
            else:
                sorted_list.append(boundary_points[min_distance_index])
                
            #remove the nearest point to the current one as it is now in the sorted list and shouldnt be found again
            boundary_points.pop(min_distance_index)       
            
            #increment the loop counter
            i = i + 1 #move onto the next point in the sorted list


        weird_points.pop(0)

        
        print(len(weird_points))

        if plot:
            
            try:
                x = [x[0] for x in sorted_list]
                y = [x[1] for x in sorted_list]
                plt.plot(x, y, 'or', color='r', ms=1)
            except:
                pass
            
            try:
                x1 = [x[0] for x in weird_points]
                y1 = [x[1] for x in weird_points]
                #plt.plot(x1, y1, color='y', ms=1)   
            except:
                pass
            
            try:
                xs = x[125:175]
                ys = y[125:175]
                #plt.plot(xs, ys, 'or', color='g', ms=1)
            except:
                pass
            
            plt.xlabel("s")
            plt.ylabel("$\dot{s}$")
            plt.ylim(ymin=0)
            plt.show()

        return sorted_list, weird_points
    
    
    def generate_test_admissible_region(self):
        """
            return: -
                [(s2, s1dot), ..., (sn, sndot)]
        """
        s = 0
        sdot = 0
        s_incr = 0.1
        sd_incr =  1
        """
            s_dot_max = (sdot - 0.5)**2 + 5 from s = 0 -> 1
        """
        quardratic =  mm.quardratic([0,1], [0,10], 0.5, 5, 0.1)
        #s_dot_max = [5.25, ]
        """
        while s < 1:
            while sdot < :
            s = s + s_incr 
        """
        return test_region
    

    def lipschitz_test_admissible_region(self):
        """
        -work out the relevant equation for the given point
        
        -find du/dx1, find du/dx2, raw_derivatives
        
        -store the absolute value of each abs_derivatives = [state, abs( dudx1), abs(dudx2)]
        
        -choose new point and loop :)        
        """

        print("inside")#, self.admissible_region)
        
        i = 0
        for state in  self.admissible_region:
            
            x1 = state[0]
            x2 = state[1]
            
            #obtain the relevant equations
            L_eq, U_eq  = self.calc_upper_lower_bound_equations((x1, x2))
            #evaluate the derivatives
            partial_dL_dx1_evaluated, partial_dL_dx2_evaluated \
            , partial_dU_dx1_evaluated, partial_dU_dx2_evaluated \
            = self.get_partial_derivatives(L_eq, U_eq, (x1,x2))

            #print("L derivatives: ", partial_dL_dx1_evaluated, partial_dL_dx2_evaluated)
            #print("U derivatives: ", partial_dU_dx1_evaluated, partial_dU_dx2_evaluated)
            #print("=======")
            
            if i == 0:
                partial_L = [(partial_dL_dx1_evaluated, partial_dL_dx2_evaluated, state)]
                partial_U = [(partial_dU_dx1_evaluated, partial_dU_dx2_evaluated, state)]   
                
            else:
                partial_L.append((partial_dL_dx1_evaluated, partial_dL_dx2_evaluated, state))
                partial_U.append((partial_dU_dx1_evaluated, partial_dU_dx2_evaluated, state))                
            i = i + 1
            
            
        print("derivatives calculated")
        #get a list of each
        partial_L_dx1 = [x[0] for x in partial_L]
        partial_L_dx2 = [x[1] for x in partial_L]
        partial_L = partial_L_dx1 + partial_L_dx2
        
        print(len(partial_L))
        
        partial_U_dx1 = [x[0] for x in partial_U]
        partial_U_dx2 = [x[1] for x in partial_U]
        partial_U = partial_U_dx1 + partial_U_dx2
        
        print(len(partial_U))
        
        max_gradient_L = np.amax(np.absolute(partial_L))
        max_gradient_U = np.amax(np.absolute(partial_U))

        print("L max gradient: -> ", max_gradient_L)
        print("U max gradient: -> ", max_gradient_U)


    def calc_upper_lower_bound_equations(self, point, return_value=False):
        """
        Method to take in a point and evaluate the upper and lower bounds at that point
        Arguments:
            point = (s, sd)
            
        bounds is a list of bounds on each joint [(bound 1, bound 2, directions decider)1,
                                                  (bound 1, bound 2, directions decider)2,
                                                  (bound 1, bound 2, directions decider)3,
                                                  (...)
                                                  (bound 1, bound 2, directions decider)N,]
        """

        
        bounds = self.bounds
        i = 0

        #print(L, U)
        for bound in bounds:
            #print(point)
            lim_direction_decider = bound[2].subs([(self.s, point[0]), (self.sd, point[1])])
            sub_list = [(self.s, point[0]), (self.sd, point[1])]
            

            try:
                #put the bounds the right way round
                if lim_direction_decider > 0:
                    L_to_check =  bound[0].subs(sub_list)
                    U_to_check = bound[1].subs(sub_list)
                    
                elif lim_direction_decider < 0:
                    L_to_check =  bound[1].subs(sub_list)
                    U_to_check = bound[0].subs(sub_list)
                    
                else:
                    raise Exception("M(s) cannot be equal to zero - error in calc_upper_lower_bound_values method")
                
                #first loop set L as the first bounds
                if i == 0:
                    L = L_to_check
                    U = U_to_check
                    #store the corresponding equations in variables
                    if lim_direction_decider > 0:
                        L_eq =  bound[0]
                        U_eq = bound[1]
                        
                    elif lim_direction_decider < 0:
                        L_eq =  bound[1]
                        U_eq = bound[0]                  

                #other loops decide where we have found L or U
                else:
                    if L_to_check > L:
                        L = L_to_check
                        #save the corresponding equation
                        if lim_direction_decider > 0:
                            L_eq =  bound[0]
                        elif lim_direction_decider < 0:
                            L_eq =  bound[1]
                    if U_to_check < U:
                        U = U_to_check
                        #save the corresponding equation
                        if lim_direction_decider > 0:
                            U_eq = bound[1]    
                        elif lim_direction_decider < 0:
                            U_eq = bound[0]   
            except:
                print("weird direction decider = ", lim_direction_decider)
                pass
            i = i + 1
            #print(i)
            
        if return_value:  
            return L, U
        else:
            return L_eq, U_eq
        
    def get_partial_derivatives(self, U_eq, L_eq, state):
        """
        function to get the partial derivatives of the input functions evaluated at the state
        """

        x1 = state[0]
        x2 = state[1]
        
        #obtain the relevant equations
        L_eq, U_eq  = self.calc_upper_lower_bound_equations((x1, x2))
        
        #get the derivatives for the L equations
        #==============================================================
        #sub in the x1 point and get the derivative wrt x2      
        L_eq_x1_subed = L_eq.subs([(self.s, x1)])
        partial_dL_dx2 = diff(L_eq_x1_subed, self.sd)
        
        #evalaute the derivative at the point of interest
        partial_dL_dx2_evaluated = partial_dL_dx2.subs([(self.sd, x2)])
        #===============================================================
        #sub in the x2 point of the same state and get the derivative wrt x1
        
        L_eq_x2_subed = L_eq.subs([(self.sd, x2)])
        partial_dL_dx1 = diff(L_eq_x2_subed, self.s)
        #evalaute the derivative at the point of interest
        partial_dL_dx1_evaluated = partial_dL_dx1.subs([(self.s, x1)])
        #================================================================
          
        
        #get the derivatives for the L equations
        #==============================================================
        #sub in the x1 point and get the derivative wrt x2      
        U_eq_x1_subed = U_eq.subs([(self.s, x1)])
        partial_dU_dx2 = diff(U_eq_x1_subed, self.sd)
        
        #evalaute the derivative at the point of interest
        partial_dU_dx2_evaluated = partial_dU_dx2.subs([(self.sd, x2)])
        #===============================================================
        #sub in the x2 point of the same state and get the derivative wrt x1
        
        U_eq_x2_subed = U_eq.subs([(self.sd, x2)])
        partial_dU_dx1 = diff(U_eq_x2_subed, self.s)
        #evalaute the derivative at the point of interest
        partial_dU_dx1_evaluated = partial_dU_dx1.subs([(self.s, x1)])
        #================================================================           
        
        return partial_dL_dx1_evaluated, partial_dL_dx2_evaluated, partial_dU_dx1_evaluated, partial_dU_dx2_evaluated
    
    
    