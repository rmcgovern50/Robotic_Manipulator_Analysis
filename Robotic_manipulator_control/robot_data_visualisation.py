# -*- coding: utf-8 -*-
"""
Created on Fri Apr  3 20:38:47 2020

@author: Ryan
This file contains a class that will be used to visualise different parts of the system
"""

import matplotlib.pyplot as plt
import numpy as np
import math as m


class robot_data_visualisation():
    
    
    def generate_state_space_plot(self, admissable_region,  save=True, marker_size=1, filepath="addmissable_plot"):
        """
        this function simply takes in a list of tuples and plots them
        Arguments:
            admissable_region [(s1,sd1), (s2,sd2), ... ,(sn,sdn)]
            
        """
        
        x_val = [x[0] for x in admissable_region]
        y_val = [x[1] for x in admissable_region]
        
        fig = plt.figure()
        ax1 = plt.subplot2grid((1,1),(0,0))
        
        ax1.plot(x_val,y_val,'or',ms=marker_size)
        plt.xlabel("s")
        plt.ylabel("$\dot{s}$")
        if save == True:
            fig.savefig("plots/"+filepath)
        else:
            plt.show()
        

    def generate_control_algorithm_plot(self, admissable_region, \
                                        control_trajectory, switching_points, \
                                        save=True, marker_size=1, filepath="s_sdot_plane.png"):
        """
        this function simply takes in two lists of tuples and plots them
        Arguments:
            admissable_region [(s1,sd1), (s2,sd2), ... ,(sn,sdn)]
            admissable_region [(s1,sd1), (s2,sd2), ... ,(sn,sdn)]

        """



        x1_val = [x[0] for x in admissable_region]
        y1_val = [x[1] for x in admissable_region]
        
        x2_val = [x[0] for x in control_trajectory]
        y2_val = [x[1] for x in control_trajectory]
        
        x3_val = [x[0] for x in switching_points]
        y3_val = [x[1] for x in switching_points]
        
        fig = plt.figure()
        
        plt.plot(x1_val, y1_val, 'or',ms=marker_size, color='r', label='admissable region')
        plt.plot(x2_val, y2_val, 'or',ms=2*marker_size, color='b', label='controlled trajectory' )
        plt.plot(x3_val, y3_val, 'or',ms=10*marker_size, color='c', label='switching points')
        
        plt.title("Plot of robot trajectory through admissable region")
        plt.legend()
        plt.xlabel("s")
        plt.ylabel("$\dot{s}$")
        if save == True:
            fig.savefig("plots/"+filepath)
        else:
            plt.show()


            
    def plot_q1_against_s(self, s_axisq1, save=True):
        
            x_val = [x[0] for x in s_axisq1]
            y_val = [x[1] for x in s_axisq1]

            y_val = np.rad2deg(np.array(y_val, dtype=np.float32))
        
            fig = plt.figure()
            plt.plot(x_val, y_val,'or',ms=5)
            
            plt.grid(color='black', linestyle='-', linewidth=0.5)
            plt.title("Angle of q1 vs s")
            plt.xlabel("s")
            plt.ylabel("q1 (degrees)")
            if save == True:
                fig.savefig("plots/q1 vs s.png")
            else:
                plt.show()
    
    def plot_q2_against_s(self, s_axisq2, save=True):
        
            x_val = [x[0] for x in s_axisq2]
            y_val = [x[1] for x in s_axisq2]
            
            y_val = np.rad2deg(np.asarray(y_val, dtype=np.float32))
            
            fig = plt.figure()
            plt.plot(x_val,y_val,'or',ms=5)
            
            plt.grid(color='black', linestyle='-', linewidth=0.5)
            plt.title("Angle of q2 vs s")
            plt.xlabel("s")
            plt.ylabel("q2 (degrees)")
            if save == True:
                fig.savefig("plots/q2 vs s.png")
            else:
                plt.show() 
            
    def plot_q1_against_q2(self, coordinates, save=True):
        
            x_val = [x[0] for x in coordinates]
            y_val = [x[1] for x in coordinates]
            
            x_val = np.rad2deg(np.asarray(x_val, dtype=np.float32))
            y_val = np.rad2deg(np.asarray(y_val, dtype=np.float32))
            
            fig = plt.figure()
            plt.plot(x_val,y_val,'or',ms=5)
            plt.grid(color='black', linestyle='-', linewidth=0.5)
            plt.title("Angle of q1 vs angle of q2")
            plt.xlabel("q1 (degrees)")
            plt.ylabel("q2 (degrees)")
            if save == True:
                fig.savefig("plots/q1 vs q2.png")
            else:
                plt.show()
        
        
    def plot_end_effector_x_y(self, x_y_coordinates, s_axisq1, save=True):
           
            x_val = [x[0] for x in x_y_coordinates]
            y_val = [x[1] for x in x_y_coordinates]
            
            fig = plt.figure()
            plt.plot(x_val,y_val,'or', color='orange', ms=5, label="end effector")
            i = 0
            
            
            q1 = [x[1] for x in s_axisq1]
            for x in x_val:
                if i == 0:
                    #attach labels on the first loop
                    plt.plot([self.link_lengths[0]*m.cos(q1[i]), x],\
                              [self.link_lengths[0]*m.sin(q1[i]), y_val[i]], '-', color='g', label="links")
                    
                    plt.plot([0, self.link_lengths[0]*m.cos(q1[i])],\
                              [0, self.link_lengths[0]*m.sin(q1[i])], '-', color='g')
                    
                    plt.plot(self.link_lengths[0]*m.cos(q1[i]),\
                              self.link_lengths[0]*m.sin(q1[i]), 'or', color='b', ms=2, label="joint")
                else:
                    
                    plt.plot([self.link_lengths[0]*m.cos(q1[i]), x],\
                              [self.link_lengths[0]*m.sin(q1[i]), y_val[i]], '-', color='g')
                    
                    plt.plot([0, self.link_lengths[0]*m.cos(q1[i])],\
                              [0, self.link_lengths[0]*m.sin(q1[i])], '-', color='g')
                    
                    plt.plot(self.link_lengths[0]*m.cos(q1[i]),\
                              self.link_lengths[0]*m.sin(q1[i]), 'or', color='b', ms=2)
                i = i + 1
                
            plt.grid(color='black', linestyle='-', linewidth=0.5)
            plt.legend()
            
            
            #scale axis
            max_extension = self.link_lengths[0] + self.link_lengths[1] + 0.2
            plt.xlim(right=max_extension) #xmax is your value
            plt.xlim(left=-max_extension) #xmin is your value
            plt.ylim(top=max_extension) #ymax is your value
            plt.ylim(bottom=-max_extension) #ymin is your value
            
            plt.title("Workspace_trajectory")
            plt.xlabel("x (metres)")
            plt.ylabel("y (metres)")
            if save == True:
                fig.savefig("plots/workspace.png")
            else:
                plt.show()      
