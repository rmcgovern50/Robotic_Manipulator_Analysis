from math import sin, cos, tan, pi
from sympy import simplify, symbols, Eq, Matrix
import numpy as np
from my_sorting import combine_to_tuples, order_states_by_x1
from intersection_finder import intersection
import matplotlib.pyplot as plt


def sec(x):

    return 1/cos(x)

def cosec(x):

    return 1/sin(x)

def cot(x):
    
    return 1/tan(x)



def compare_expression(expression1, expression2):
    """
    simple function to check if two expressions are the same
    return true if they are and also return the difference
    
    Arguments:
    expression1 - sympy expression
    expression2 - sympy expression
    
    return:
    true - if both expressions are equivalent
    false - if expressions are different
    """
       
    difference = simplify(expression1 - expression2)
    
    if simplify(expression1 - expression2):
        return True
    else:
        return False
    
    
def sub_into_system_of_equs(equation_list, vars_to_sub):
    """
    simple function that takes 2 lists
    
    Arguments:
    equation_list - list of sympy equations
    vars_to_sub - list of N tuples of length 2 [(varname1,val1),...,(varnameN,valN)]
    
    return - list of sympy equations of equal dimension to equation_list
    """
    i = 0
    #
    
    
    for equ in equation_list:
    #cycle through each equation and substitute                
        if i == 0:
            new_equation_list = [Eq((equ.lhs).subs(vars_to_sub), (equ.rhs).subs(vars_to_sub))]
            
        else:
            new_equation_list.append(Eq((equ.lhs).subs(vars_to_sub), (equ.rhs).subs(vars_to_sub)))
        i = i + 1

    return new_equation_list
        


def sub_into_matrix(matrix, vars_to_sub):
    """
    Arguments:
    matrix - sympy matrix or nxm dimensions where n and m can be and positive integer 
    vars_to_sub - list of N tuples of length 2 [(varname1,val1),...,(varnameN,valN)]
    
    return - sympy matrix with desired values subed into each element of equal dimension to equation_list
    """
    
    rows = matrix.shape[0]
    cols = matrix.shape[1]  
    #print(rows, cols)
    newM = []
    #print(new_matrix)
    
    i = 0
    j = 0
    
    #count down through rows
    while(i < rows):
        #count along columns in a given row
        while(j < cols):
            if j == 0:
                row = [matrix.row(i)[j].subs(vars_to_sub)]
            else:
                row.append(matrix.row(i)[j].subs(vars_to_sub))
                
            #print(matrix.row(i)[j].subs(vars_to_sub))
            j = j + 1
        
        #store rows in a form to preserve the matrix structure
        if i == 0:
            newM = [row]
        else:
            newM.append(row)
        i = i + 1
        j = 0

    
    new_matrix = Matrix(newM)#put the list back in matrix form


    return new_matrix


def fit_curve(tuple_list, degree=1):
    """
    this function simply takes in a list of tuples and fits a curve to them
    Arguments:
        tuple_list [(x1,y1), (x2,y2), ... ,(xn,yn)]
    
    """
    x_val = [x[0] for x in tuple_list]
    y_val = [x[1] for x in tuple_list]
         
    #convert the lists to numpy
    x_val = np.array(x_val, dtype=float)
    y_val = np.array(y_val ,dtype=float)
    
    z = np.polyfit(x_val,y_val,1)
        
    p1 = np.poly1d(z)
    return x_val, p1(x_val)
    
    #plt.plot(n, p1(x_val))



def rotate_xy(vector, angle):
    """
    function that takes in a 3D vector and rotates the x and y points by some
    specified angle given in radians
    
    Arguments:
        vector - [x, y, z]
        angle - angle in radians
    
    return:
        rotated_vector - new_vector that is rotated as a list [x, y z]
    
    """
    a = angle
    rotation_marix = np.array([[cos(a), -1*sin(a), 0], [sin(a), cos(a), 0], [0,0,1]])
    
    xyz_old = np.array([[vector[0]],[vector[1]],[vector[2]]])
    xyz_new = rotation_marix.dot(xyz_old)
    
    rotated_vector = [xyz_new[0][0], xyz_new[1][0], xyz_new[2][0]]
    
    #print("New vector", xyz_new)
    
    return rotated_vector  


def find_intersection_points(first_seg, second_seg, p1_order=20, p2_order=20 ):
    """
    This function takes in 2 segments and finds the intesection
    return value gives intersection points from left to right
    """
    #print("searching for intersections...")
    import warnings
    warnings.filterwarnings("ignore")

    x1 = [x[0] for x in first_seg]
    y1 = [x[1] for x in first_seg]
    
    x2 = [x[0] for x in second_seg]
    y2 = [x[1] for x in second_seg]
    
    dt = np.dtype(np.complex128) # 128-bit complex floating-point number
    x1 = np.array(x1, dtype=dt)
    y1 = np.array(y1, dtype=dt)
    x2 = np.array(x2, dtype=dt)
    y2 = np.array(y2, dtype=dt)  

    
    z1 = np.polyfit(x1,y1, p1_order) #x and y describe function to be approximated, the number is the order of polynomial
    z2 =  np.polyfit(x2,y2, p2_order)
        
    xint, yint = intersection(x1, y1, x2, y2)
    #print(type(xint))
    if len(xint) == 0 or len(yint) == 0:
        return False, False
    else:
        return xint, yint


def find_closest_points_in_data(trajectory_1, trajectory_2, plot=False):
    """
    This function takes in 2 trajectories and returns the index of the closest points

    Parameters
    ----------
    trajectory_1 : [(x1,y1),...,(xn, yn)]
        DESCRIPTION.
    trajectory_2 : [(x1,y1),...,(xn, ym)]
        DESCRIPTION.

    Returns
    -------
    T1_in : integer 
        index of trajectory 1 where the closest point is found
    T2_in : integer
        index of trajectory 2 where the closest point is found
    """
    
    
    x1list = [x[0] for x in trajectory_1]
    y1list = [x[1] for x in trajectory_1]
        
    x2list = [x[0] for x in trajectory_2]
    y2list = [x[1] for x in trajectory_2]
    
    x1 = np.array(x1list, dtype=np.float32)
    y1 = np.array(y1list, dtype=np.float32)

    x_points_traj_2 = np.array(x2list, dtype=np.float32)
    y_points_traj_2 = np.array(y2list, dtype=np.float32)
    
    traj_1_index = 0
    best_closest_point_dist = 1000#initialise as a very high number
    
    #find the points in trajectory 2 that are closest esch point in trajectory 1 
    for point in trajectory_1:
        x_point_bulked = point[0]*np.ones(len(x_points_traj_2))
        y_point_bulked = point[1]*np.ones(len(y_points_traj_2))
        
        distx = x_point_bulked - x_points_traj_2
        disty = y_point_bulked - y_points_traj_2

        dist = np.square(distx) + np.square(disty)
        #list_of_distances.append(dist)
        
        closest_point_dist = np.min(dist)
        traj_2_index = np.argmin(dist)
        
        if closest_point_dist < best_closest_point_dist:
            best_closest_point_dist = closest_point_dist
            best_point_indices = (traj_1_index, traj_2_index)
        
        traj_1_index = traj_1_index + 1
        
    #print(best_closest_point_dist, best_point_indices)
    
    T1_in = best_point_indices[0]
    T2_in = best_point_indices[1]
    
    #print("closest_points", trajectory_1[T1_in], trajectory_2[T2_in])
    
    if plot==True: 
        fig = plt.figure()    
        ax1 = plt.subplot2grid((1,1),(0,0))
        ax1.plot(x1, y1,'or',ms=2, color="r")
        ax1.plot(x2list, y2list,'or',ms=2, color="b")
        ax1.plot(trajectory_1[T1_in][0],trajectory_1[T1_in][1],'x',ms=10, color="r")
        ax1.plot(trajectory_2[T2_in][0],trajectory_2[T2_in][1],'x',ms=10, color="b")
        plt.xlabel("x")
        plt.ylabel("y")
        plt.axis('equal')
        plt.grid()
        plt.show()           
     
    return T1_in, T2_in
  
def find_intersect_in_trajectories(trajectory_1, trajectory_2, plot=False, \
                                   return_index=False, epsilon = 0.01):
    
    """
    The epsilon value allows the maximum gap between points to be set before its
    not counted as in intesect, this may take some experimentation to set
    """
        
    T1 = trajectory_1
    T2 = trajectory_2
    
    #get the indexs of the closest points in the data
    T1_index, T2_index = find_closest_points_in_data(T1, T2)    
    #print("T1 close state", T1[T1_index])
    #print("T2 close state", T2[T2_index])
    #print(T1_index, T2_index)

    delta_x1 = abs(T1[T1_index][0] - T2[T2_index][0])
    delta_x2 = abs(T1[T1_index][1] - T2[T2_index][1])
    #epsilon = 0.1

    #print(delta_x1, delta_x2)
    if delta_x1 < epsilon and delta_x2 < epsilon:
        #ensure it is not the first or last element
        if T1_index != (len(trajectory_1)-1) and T1_index > 0:
            T1_points = [T1[T1_index-1],T1[T1_index],T1[T1_index+1]]
        #the first element happens to be the one closest 
        elif T1_index == 0:
            T1_points = [T1[T1_index],T1[T1_index+1],T1[T1_index+2]]
        #if it is the last element        
        elif T1_index == (len(trajectory_1)-1):
            T1_points = [T1[T1_index-2],T1[T1_index-1],T1[T1_index]]        
    
        #ensure it is not the first or last element of t1
        if T2_index != (len(trajectory_2)-1) and T2_index > 0:
            T2_points = [T2[T2_index-1],T2[T2_index],T2[T2_index+1]]
        #the first element happens to be the one closest 
        elif T2_index == 0:
            T2_points = [T2[T2_index], T2[T2_index+1], T2[T2_index+2]]
        #if it is the last element
        elif T2_index == (len(trajectory_2)-1):
            T2_points = [T2[T2_index-2],T2[T2_index-1],T2[T2_index]]
        
        #print("points near intersection are ", T1_points, T2_points)
        
        
        xa = [x[0] for x in T1_points] 
        ya = [x[1] for x in T1_points]
        
        xb = [x[0] for x in T2_points]
        yb = [x[1] for x in T2_points]
        
        
        xa= np.array(xa,dtype=np.float32)
        ya= np.array(ya,dtype=np.float32)    
        xb= np.array(xb,dtype=np.float32)
        yb= np.array(yb,dtype=np.float32)    
        
        #just use linear approximation for simplicity    
        z1 = np.polyfit(xa,ya, 1) 
        z2 =  np.polyfit(xb,yb, 1)    
        
        #get range
        min_x = min([min(xa), min(xb)])
        max_x = max([max(xa), max(xb)])
        
        x_intersect_range = np.linspace(min_x, max_x, 100)        
        
        #get all the roots
        roots_found = np.roots(z1-z2)
        #get all the real roots
        real_roots = roots_found[np.isreal(roots_found)]
        #print("number of real roots ", len(real_roots))
        #print(real_roots)
        #print(min_x, max_x)
    #
        i = len(real_roots)-1
        
        potential_roots = [0]
        
        #save all the feasible roots
        while i >=0:
            #if root lies in the correct range save it
            if real_roots[i] < max_x and real_roots[i] > min_x:
                potential_roots.append(real_roots[i])    
            i = i-1
        
        potential_roots.pop(0)
        print("potential roots ", potential_roots)
            
        try:
            #incase a polynomial order > 1 is used just take the lowest x value
            #this is a lazy solution and may need work            
            x_intersect = min(potential_roots)
            y_intersect = np.polyval(z1, x_intersect)
            
            intersection = (x_intersect, y_intersect)
        except:
            intersection = False
        #print(intersection)
        
        
        if plot == True:
            
            x1 = [x[0] for x in trajectory_1]
            y1 = [x[1] for x in trajectory_1]
                
            x2 = [x[0] for x in trajectory_2]
            y2 = [x[1] for x in trajectory_2]
             
            ax1 = plt.subplot2grid((1,1),(0,0))
            #ax1.axis('equal')
            ax1.plot(x1, y1,'x',ms=2, color="r")
            ax1.plot(x2, y2,'x',ms=2, color="b")
            ax1.plot(trajectory_1[T1_index][0],trajectory_1[T1_index][1],'x',ms=10, color="r")
            ax1.plot(trajectory_2[T2_index][0],trajectory_2[T2_index][1],'x',ms=10, color="b")
            #ax1.plot(x_intersect_range, np.polyval(z1, x_intersect_range))
            #ax1.plot(x_intersect_range, np.polyval(z2, x_intersect_range))
            try:
                ax1.plot(intersection[0],intersection[1], 'or', color='g', ms=5)
            except:
                pass
            plt.xlabel("x")
            plt.ylabel("y")
            #plt.axis('equal')
            plt.grid()
            plt.show()
            
    else:
        #return all false to signify no intersection in data
        
        return False, False, False
    
    """
    #if we want the indexs of T1 and T2 that were found to be closest set 
    return index to True
    """         
    if return_index:
        return intersection, T1_index, T2_index
    else:
        return intersection


def find_all_intersections(T1, T2,  with_index=False, res=0.01):
    """
    use the find_intersect_in_trajectories function repetitively to find all
    the intersections that exist and return the list
    """

    #find the first intersection
    intersection, T1_index, T2_index = \
        find_intersect_in_trajectories(T1, T2, plot=True, return_index=True, epsilon=res)   
    #print("before pop", T1[T1_index], T2[T2_index])
        
    index_list = [(T1_index, T2_index)]
    #remove the closest points from the trajectories
    T1.pop(T1_index)
    T2.pop(T2_index)
    #print("after pop", T1[T1_index], T2[T2_index])    
    all_intersections = [intersection]

    
    while intersection != False:
        
        intersection, T1_index, T2_index = \
        find_intersect_in_trajectories(T1, T2, plot=True, return_index=True, epsilon=res) 

        if intersection != False:
            all_intersections.append(intersection)
            index_list.append((T1_index, T2_index))
            T1.pop(T1_index)
            T2.pop(T2_index)
 


    #print("ordered states", ordered_intersections)
    
    if with_index:
        print("the vector lengths", len(all_intersections), len(index_list))
        ordered_intersections, ordered_index = order_states_by_x1(all_intersections,\
                                                                  supporting_list=index_list)
        return ordered_intersections, ordered_index 
    else:
        ordered_intersections = order_states_by_x1(all_intersections)
        return all_intersections 


def min_function_two_constraints(T1, T2):
    """
       find all intersections outputs an ordered list of intersection points from left to right 
    """
    intersection_list, index_list = find_all_intersections(T1, T2, with_index=True)
    #print(intersection_list)
    #print(index_list)
    i = 0
    start_index_T1 = 0
    start_index_T2 = 0
        
    min_data = []
    #now loop through and work out which function segments are required
    for intersect, index in zip(intersection_list, index_list):
        T1_index = index[0]
        T2_index = index[1]
        x1_intersect = intersect[0]
        #print("int, T1i, T2i", intersect, T1_index, T2_index)
        """
        we need to work out which side of the intersection each indexed point lies
        """
        #print(x1_intersect, T1[T1_index][0], T2[T2_index][0])
        
        """
        first find T1_index_before and T1_index_after
        """
        try:
            j = T1_index
            while T1[j][0] > x1_intersect and j >= 0:
                j = j - 1  
                
            T1_index_before = j
    
            j = T1_index
            while T1[j][0] < x1_intersect and j < len(T1)-1:
                j = j + 1
            T1_index_after = j        
           
            """
            repeat for the second trajectory
            """
            j = T2_index
            while T2[j][0] > x1_intersect and j >= 0:
                j = j - 1    
            T2_index_before = j
    
            j = T2_index
            while T2[j][0] < x1_intersect and j < len(T2)-1:
                j = j + 1    
            T2_index_after = j
        except:
            #worst case just stick with the index given
            T1_index_before = T1_index
            T1_index_after = T1_index
            T2_index_before = T2_index
            T2_index_after = T2_index
            
        """
        extract the important states, the x2 values of before and
        after states can be chosen to tell us what sections of trajectories to 
        keep
        """
        print("T1_index before ", T1_index_before)
        print("length T1 ",  len(T1))
        try:
            T1_before = T1[T1_index_before] 
            T2_before = T2[T2_index_before] 
        except:
            break
        #print("T1 before", T1_before[1], "T2 before",T2_before[1])
        #print("i", i)
        
        if i == 0:
            #work out which data to keep before the intersection
            if T1_before[1] < T2_before[1]:
                #T1 should be kept from the start index until T1_index_before
                min_data = [T1[start_index_T1:T1_index_before]]
            else:
                min_data = [T2[start_index_T2:T2_index_before]]           
        else:
            if T1_before[1] < T2_before[1]:
                #T1 should be kept from the start index until T1_index_before
                min_data.append(T1[start_index_T1:T1_index_before])
            else:
                min_data.append(T2[start_index_T2:T2_index_before])
                
        #save the start index for the next loop
        start_index_T1 = T1_index_after
        start_index_T2 = T2_index_after          
        
        i = i + 1
        
    """
    Now the last important segment should be selected 
    """
    #print(start_index_T1, start_index_T2, T1_index_after, T2_index_after)
    T1_after = T1[start_index_T1]
    T2_after = T2[start_index_T2]
    
    if T1_after[1] < T2_after[1]:
        #T1 should be kept from the start index until T1_index_before
        min_data.append(T1[start_index_T1:])
    else:
        min_data.append(T2[start_index_T2:])    
    
    #print(len(min_data))
    return min_data


#enable to test anything
if __name__ == "__main__": 

    x1 = [1, 2, 3 , 4, 5]
    y1 =  [10, 8, 6, 4, 2]
    #y1 =  np.random.randint(2, size=5)#
    
    x2 = [1.23, 2.3, 4.8, 5.9, 7]
    y2 =  [1, 5, 5, 6.1, 6.9]
    
    x1 = np.linspace(0, 1, 10000)
    x2 = np.linspace(0, 1, 6800)
    #y = np.linspace(8, 8, 50)      

    y1 = (2*(x1-0.5))**2 +9
    #y2 = -1*((8*(x2-0.5))**2 -19)
    #y2 =  4*np.sin(10*(x2)**2) + -2*np.sin(18*x2 + x2) + np.exp(-1*x2) + x2 + 5*x2**2 + 3    
    y2 = 4*np.sin(10*x2) + 2*np.sin(18*2) + 10      
    T1 = combine_to_tuples(x1, y1)
    T2 = combine_to_tuples(x2, y2)
       
    min_data = min_function_two_constraints(T1, T2)
    
    #all_intersections = find_all_intersections(lin, quad)
    #print("resulting intersections", all_intersections)
    #find_intersect_in_trajectories(lin, quad, plot=True)        
    
    #zlin = np.polyfit(xlin,ylin, 20) #x and y describe function to be approximated, the number is the order of polynomial
    #zquad =  np.polyfit(xquad,yquad, 20)
    
    
    
    #ylin_calc = np.polyval(zlin, xlin)
    #yquad_calc = np.polyval(zquad, xquad)
    
    
    #xint, yint = intersection(xlin, ylin, xquad, yquad)
    
    #print(xint, yint)

    fig = plt.figure()
        
    ax1 = plt.subplot2grid((1,1),(0,0))
    
    ax1.plot(x1, y1,'or',ms=5, color="r")
    ax1.plot(x2, y2,'or',ms=5, color="b")
    
    for min_data_seg in min_data:
        
        x1_min = [x[0] for x in min_data_seg]
        x2_min = [x[1] for x in min_data_seg ]
        
        ax1.plot(x1_min, x2_min,'or',ms=5, color="y")
    """
    x1_min_2 = [x[0] for x in min_data[1]] 
    x2_min_2 = [x[1] for x in min_data[1]] 
    
    ax1.plot(x1_min_2, x2_min_2,'or',ms=5, color="y")
    
    x1_min_3 = [x[0] for x in min_data[2]] 
    x2_min_3 = [x[1] for x in min_data[2]] 
    
    ax1.plot(x1_min_3, x2_min_3,'or',ms=5, color="y")
    """
    #ax1.plot(xlin, ylin_calc, color= "g")
    #ax1.plot(xquad, yquad_calc, color="c")
    
    #ax1.plot(xint[0], yint[1], color="r")
    
    plt.xlabel("x")
    plt.ylabel("y")
    plt.show()

    