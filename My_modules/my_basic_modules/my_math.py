from math import sin, cos, tan, pi
from sympy import simplify, symbols, Eq, Matrix
import numpy as np

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


def intersection(p1, p2):
    """
    this function takes in two lines and returns the intersection of the two    
    """
    x = np.roots(p1-p2)
    y = np.polyval(p1, x)
    print(x, y)
    intersection = (x[0], y[0])
    
    return intersection


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
    
    
#enable to test anything
if __name__ == "__main__": 
    vector = [1,0,0]
    angle = -pi/2
    rotate_xy(vector, angle)

        
    
    
    