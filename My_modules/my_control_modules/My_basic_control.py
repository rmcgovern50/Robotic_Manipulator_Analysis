# -*- coding: utf-8 -*-
"""
Created on Sun Mar 15 13:57:11 2020
This is a simple module that performs basic control engineering tasks
"""

#from scipy import signal
#from sympy import Matrix



import numpy as np
import matplotlib.pyplot as plt

def f(Y, t):
    y1, y2 = Y
    return [y2, -np.sin(y1)]
    


y1 = np.linspace(1, 4, 4)#create a line list of points y1 1 - 4 in 4 steps
y2 = np.linspace(1, 2, 4)
print(y1, y2)
print("==========")

Y1, Y2 = np.meshgrid(y1, y2) #maoke rectangular mesh that's plotable for y1, y2 
print(Y1)
print("==========")
print(Y2)

plt.scatter(Y1, Y2)
plt.show()


t = 0

u, v = np.zeros(Y1.shape), np.zeros(Y2.shape)

NI, NJ = Y1.shape

for i in range(NI):
    for j in range(NJ):
        x = Y1[i, j]
        y = Y2[i, j]
        yprime = f([x, y], t)
        u[i,j] = yprime[0]
        v[i,j] = yprime[1]
     

Q = plt.quiver(Y1, Y2, u, v, color='r')

plt.xlabel('$y_1$')
plt.ylabel('$y_2$')
plt.xlim([-2, 8])
plt.ylim([-4, 4])






"""

def plot_linear_sys_ss(A, B, C, D):
   
    Arguments:
        A, B, C and D matrices (sympy matrix) from a linear system of form:
        xd = A*x +B*u,     y = C*x + D*u

    sys = StateSpace(A,B,C,D)
    #step_reponse(sys)
    #print()
   
if __name__ == "__main__": 
    A= Matrix([[1,1],[1,1]])
    B= Matrix([[0],[0]])
    C= Matrix([[1,1],[1,1]])
    D= Matrix([[0],[0]])
    
    plot_linear_sys_ss(A,B,C,D)
    
"""