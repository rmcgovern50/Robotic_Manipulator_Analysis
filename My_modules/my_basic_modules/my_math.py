from math import sin, cos, tan
from sympy import simplify, symbols, Eq, Matrix


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

        
        
        
#enable to test anything
if __name__ == "__main__": 
    
    
    """
    x, y, z = symbols('x, y z')
    list_of_equations = [Eq(x + y, z),\
                         Eq(x + y**2, z**2),\
                         Eq(x, y + 2*z**2)]
    
    constants_to_sub = [(z, 5), (x, 3)]
    
    
    m = Matrix([[x+y,x+z],[y+x+2*z,x**2 + z**2 + x],[y-x+2*z,x + 2*z**2 + x] ])
    new_m= sub_into_matrix(m, constants_to_sub)
    print(new_m)
    #print(sub_into_system_of_equs(list_of_equations, constants_to_sub))
    """ 
        
        
        
        
        
    
    
    