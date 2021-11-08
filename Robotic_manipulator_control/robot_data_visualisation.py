# -*- coding: utf-8 -*-
"""
Created on Fri Apr  3 20:38:47 2020

@author: Ryan
This file contains a class that will be used to visualise different parts of the system
"""

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d import axes3d
import numpy as np
import math as m
#import plots.png_to_gif as png_to_gif
import datetime as dt

from labellines import *


class two_dof_robot_data_visualisation():
    
    def __init__(self, current_time):
        self.time = current_time
    
    def generate_state_space_plot(self, admissible_region,  save=True, marker_size=1, filename="admissible_plot", filepath="admissible_region/", title="undefined"):
        """
        this function simply takes in a list of tuples and plots them
        Arguments:
            admissable_region [(s1,sd1), (s2,sd2), ... ,(sn,sdn)]
            
        produces a plot of the s sdot region
            
        """
        
        x_val = [x[0] for x in admissible_region]
        y_val = [x[1] for x in admissible_region]
        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
            
        ax1 = plt.subplot2grid((1,1),(0,0))
        ax1.plot(x_val,y_val,'or',ms=marker_size)
        plt.title(title)
        plt.xlabel("$X_1$")
        plt.ylabel("$X_2$")
        if save == True:
            fig.savefig("plots/" +filepath + filename + self.time)
            plt.close()
        else:
            plt.show()
        

    def generate_control_algorithm_plot(self, admissable_region, \
                                        control_trajectory, switching_points, \
                                        save=True, marker_size=1, filepath="s_sdot_plane"):
        """
        this function simply takes in two lists of tuples and plots them
        Arguments:
            admissable_region [(s1,sd1), (s2,sd2), ... ,(sn,sdn)]
            admissable_region [(s1,sd1), (s2,sd2), ... ,(sn,sdn)]
        
        produces a plot of the s sdot region with a control trajectory and switchng pounts marked

        """
        plot_switch = True

        x1_val = [x[0] for x in admissable_region]
        y1_val = [x[1] for x in admissable_region]
        
        x2_val = [x[0] for x in control_trajectory]
        y2_val = [x[1] for x in control_trajectory]
        try:
            x3_val = [x[0] for x in switching_points]
            y3_val = [x[1] for x in switching_points]
        except:    
            print("no valid switching points")
            plot_switch = False
        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
        
        plt.plot(x1_val, y1_val, 'or',ms=0.25*marker_size, color='r', label='admissable region')
        plt.plot(x2_val, y2_val, 'or',ms=1*marker_size, color='b', label='controlled trajectory' )
        
        if plot_switch == True:
            plt.plot(x3_val, y3_val, 'or',ms=10*marker_size, color='c', label='switching points')
        
        plt.title("Plot of robot trajectory through admissable region")
        plt.legend()
        plt.xlabel("s")
        plt.ylabel("$\dot{s}$")
        #print("hvsbhvefb")
        if save == True:
            fig.savefig("plots/control_trajectories/"+filepath + self.time)
            plt.close()
        else:
            plt.show()

    def generate_overlayed_polynomial_control_algorithm_plot(self, z, x, \
                                        admissible_region, control_trajectory, switching_points, \
                                        save=True, marker_size=1, filepath="s_sdot_plane_with_control_overlayed"):
 
        plot_switch = True

        x1_val = x
        y1_val = np.polyval(z, x)
        
        x2_val = [x[0] for x in control_trajectory]
        y2_val = [x[1] for x in control_trajectory]
        
        x4_val = [x[0] for x in admissible_region]
        y4_val = [x[1] for x in admissible_region]
        
        try:
            x3_val = [x[0] for x in switching_points]
            y3_val = [x[1] for x in switching_points]
        except:    
            print("no valid switching points")
            plot_switch = False
        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
            
        plt.plot(x4_val, y4_val, 'or', ms=1*marker_size, color='r', label='admissable region')
        plt.plot(x1_val, y1_val, ms=1*marker_size, color='c', label='artifical contraint boundary')
        plt.plot(x2_val, y2_val, 'or',ms=1*marker_size, color='g', label='controlled trajectory' )
        
        if plot_switch == True:
            plt.plot(x3_val, y3_val, 'v',ms=5*marker_size, color='b', label='switching points')
        
        plt.title("Plot of robot trajectory through admissable region")
        plt.legend()
        plt.grid()
        plt.xlabel("s")
        plt.ylabel("$\dot{s}$")
        #print("hvsbhvefb")
        if save == True:
            fig.savefig("plots/polynomial_control_trajectories_overlay/"+filepath + self.time)
            plt.close()
        else:
            plt.show()       

    def generate_polynomial_control_algorithm_plot(self, z, x, \
                                        control_trajectory, switching_points, \
                                        save=True, marker_size=1, filepath="s_sdot_plane_with_control"):
 
        plot_switch = True

        x1_val = x
        y1_val = np.polyval(z, x)
        
        x2_val = [x[0] for x in control_trajectory]
        y2_val = [x[1] for x in control_trajectory]
        try:
            x3_val = [x[0] for x in switching_points]
            y3_val = [x[1] for x in switching_points]
        except:
            print("no valid switching points")
            plot_switch = False
        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
        
        plt.plot(x1_val, y1_val, ms=1*marker_size, color='r', label='admissable region boundary')
        plt.plot(x2_val, y2_val, 'or',ms=1*marker_size, color='g', label='controlled trajectory' )
        
        if plot_switch == True:
            plt.plot(x3_val, y3_val, 'v',ms=5*marker_size, color='r', label='switching points')
        
        plt.title("Plot of robot trajectory through admissable region")
        plt.legend()
        plt.grid()
        plt.xlabel("s")
        plt.ylabel("$\dot{s}$")
        #print("hvsbhvefb")
        if save == True:
            fig.savefig("plots/polynomial_control_trajectories/"+filepath + self.time)
            plt.close()
        else:
            plt.show()       
        

            
    def plot_q1_against_s(self, s_axisq1, save=True, marker_size = 5, file_name="q1 vs s.png"):
        """
        produce a plot of q1 against s
        """
        
        x_val = [x[0] for x in s_axisq1]
        y_val = [x[1] for x in s_axisq1]

        y_val = np.rad2deg(np.array(y_val, dtype=np.float32))
    
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
        plt.plot(x_val, y_val,'or',ms=marker_size)
        
        plt.grid(color='black', linestyle='-', linewidth=0.5)
        plt.title("Angle of q1 vs s")
        plt.xlabel("s")
        plt.ylabel("q1 (degrees)")
        if save == True:
            fig.savefig("plots/q1_vs_s/" + file_name + self.time)
            plt.close()
        else:
            plt.show()
 
    
    
    def plot_q2_against_s(self, s_axisq2, save=True, marker_size = 5, file_name="q2 vs s.png"):
        """
        produce a plot of q2 against s
        """
        
        
        x_val = [x[0] for x in s_axisq2]
        y_val = [x[1] for x in s_axisq2]
        
        y_val = np.rad2deg(np.asarray(y_val, dtype=np.float32))
        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
        plt.plot(x_val,y_val,'or',ms=marker_size)
        
        plt.grid(color='black', linestyle='-', linewidth=0.5)
        plt.title("Angle of q2 vs s")
        plt.xlabel("s")
        plt.ylabel("q2 (degrees)")
        if save == True:
            fig.savefig("plots/q2_vs_s/" + file_name + self.time)
            plt.close()
        else:
            plt.show() 
            
    def plot_q1_against_q2(self, coordinates, save=True, marker_size = 5, file_name="q2 vs s.svg", filepath="q1_vs_q2/", title="Joint Space"):
        """
        produce a plot of q1 against q2
        """        
        x_val = [x[0] for x in coordinates]
        y_val = [x[1] for x in coordinates]
        
        x_val = np.rad2deg(np.asarray(x_val, dtype=np.float32))
        y_val = np.rad2deg(np.asarray(y_val, dtype=np.float32))
        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
        
       
        percentage = 0.75
        arrowPosx = x_val[int( (len(x_val)-1)*percentage )]
        arrowPosxplus = x_val[int( (len(x_val)-1)*percentage )+2]
        arrowPosy = y_val[int( (len(y_val)-1)*percentage )]
        arrowPosyplus = y_val[int( (len(y_val)-1)*percentage )+2]
        dx = (arrowPosxplus - arrowPosx)
        dy = (arrowPosyplus - arrowPosy)
        plt.arrow(arrowPosx, arrowPosy, dx, dy, color= 'purple', width=0.65 )

        percentage = 0.1
        arrowPosx = x_val[int( (len(x_val)-1)*percentage )]
        arrowPosxplus = x_val[int( (len(x_val)-1)*percentage )+2]
        arrowPosy = y_val[int( (len(y_val)-1)*percentage )]
        arrowPosyplus = y_val[int( (len(y_val)-1)*percentage )+2]
        dx = (arrowPosxplus - arrowPosx)
        dy = (arrowPosyplus - arrowPosy)
        plt.arrow(arrowPosx, arrowPosy, dx, dy, color= 'purple', width=0.65 )
        

        plt.plot(x_val,y_val,'-',ms=marker_size, color = 'purple')

        plt.plot(x_val[0],y_val[0],'or', color='blue', ms=5, label="first end effector position")
        plt.plot(x_val[-1:],y_val[-1:],'or', color='blue', ms=5, label="last effector position")  
        
    
        
        #plt.grid(color='black', linestyle='-', linewidth=0.5)
        plt.title(title)
        plt.xlabel("q1 (degrees)")
        plt.ylabel("q2 (degrees)")
        if save == True:
            fig.savefig("plots/"+ filepath + file_name + self.time + ".svg", format="svg")
            fig.savefig("plots/"+ filepath + file_name + self.time + ".jpg", format="jpg")
            plt.close()
        else:
            plt.show()
        
        
    def plot_robot_motion_x_y(self, link_lengths, x_y_coordinates, s_axisq1, start_end_only=True, save=True, marker_size=5, file_name="robot_motion", filepath="", title="Workspace"):
        """
        produce a plot of qhow the robot moves in the workspace
        """           

        x_val = [x[0] for x in x_y_coordinates]
        y_val = [x[1] for x in x_y_coordinates]
            
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()

        i = 0
        plt.plot(x_val[1:len(x_val)-1],y_val[1:len(y_val)-1],'-', color='purple', ms=2, label="end effector")
        
        if start_end_only:

            q1 = [s_axisq1[0][1], s_axisq1[-1][1]] 
            #print(q1)
            
            #plot initial condition
            plt.plot([link_lengths[0]*m.cos(q1[0]), x_val[0]],\
                      [link_lengths[0]*m.sin(q1[0]), y_val[0]], color='grey', label="links", linewidth=6)
            plt.plot([0, link_lengths[0]*m.cos(q1[0])],\
                      [0, link_lengths[0]*m.sin(q1[0])], color='grey', linewidth=6)
            plt.plot(link_lengths[0]*m.cos(q1[0]),\
                      link_lengths[0]*m.sin(q1[0]), 'or', color='b', ms=marker_size, label="joint")
            
            #plot final condition
            plt.plot([link_lengths[0]*m.cos(q1[1]), x_val[-1]],\
                      [link_lengths[0]*m.sin(q1[1]), y_val[-1]], '-', color='grey', label="links", linewidth=6)
            plt.plot([0, link_lengths[0]*m.cos(q1[1])],\
                      [0, link_lengths[0]*m.sin(q1[1])], '-', color='grey', linewidth=6)
            plt.plot(link_lengths[0]*m.cos(q1[1]),\
                      link_lengths[0]*m.sin(q1[1]), 'or', color='b', ms=marker_size, label="joint")
                            
            
        else:
            q1 = [x[1] for x in s_axisq1]
            for x in x_val:
                if i == 0:
                    #attach labels on the first loop
                    plt.plot([link_lengths[0]*m.cos(q1[i]), x],\
                              [link_lengths[0]*m.sin(q1[i]), y_val[i]], '-', color='g', label="links")
                    
                    plt.plot([0, link_lengths[0]*m.cos(q1[i])],\
                              [0, link_lengths[0]*m.sin(q1[i])], '-', color='g')
                    
                    plt.plot(link_lengths[0]*m.cos(q1[i]),\
                              link_lengths[0]*m.sin(q1[i]), 'or', color='b', ms=2, label="joint")
                else:
                    
                    plt.plot([link_lengths[0]*m.cos(q1[i]), x],\
                              [link_lengths[0]*m.sin(q1[i]), y_val[i]], '-', color='g')
                    
                    plt.plot([0, link_lengths[0]*m.cos(q1[i])],\
                              [0, link_lengths[0]*m.sin(q1[i])], '-', color='g')
                    
                    plt.plot(link_lengths[0]*m.cos(q1[i]),\
                              link_lengths[0]*m.sin(q1[i]), 'or', color='b', ms=2)
                i = i + 1
            
        #plt.grid(color='black', linestyle='-', linewidth=0.5)
        #plt.legend()
        
        
        plt.plot(x_val[0],y_val[0],'or', color='blue', ms=marker_size, label="first end effector position")
        plt.plot(x_val[-1:],y_val[-1:],'or', color='blue', ms=marker_size, label="last effector position")  
        #scale axis
        max_extension = link_lengths[0] + link_lengths[1] + 0.2
        #plt.xlim(right=max_extension) #xmax is your value
        #plt.xlim(left=-max_extension) #xmin is your value
        #plt.ylim(top=max_extension) #ymax is your value
        #plt.ylim(bottom=-max_extension) #ymin is your value

        #plt.plot(x_val[1:len(x_val)-1],y_val[1:len(y_val)-1],'-', color='orange', ms=2, label="end effector")
        #xrange = 1:len(x_val)-1
        percentage = 0.23
        arrowPosx = x_val[int( (len(x_val)-1)*percentage )]
        arrowPosxplus = x_val[int( (len(x_val)-1)*percentage )+2]
        arrowPosy = y_val[int( (len(y_val)-1)*percentage )]
        arrowPosyplus = y_val[int( (len(y_val)-1)*percentage )+2]
        dx = (arrowPosxplus - arrowPosx)
        dy = (arrowPosyplus - arrowPosy)
        plt.arrow(arrowPosx, arrowPosy, dx, dy, color= 'purple', width=0.003 )
        
        percentage = 0.75
        arrowPosx = x_val[int( (len(x_val)-1)*percentage )]
        arrowPosxplus = x_val[int( (len(x_val)-1)*percentage )+2]
        arrowPosy = y_val[int( (len(y_val)-1)*percentage )]
        arrowPosyplus = y_val[int( (len(y_val)-1)*percentage )+2]
        dx = (arrowPosxplus - arrowPosx)
        dy = (arrowPosyplus - arrowPosy)
        plt.arrow(arrowPosx, arrowPosy, dx, dy, color= 'purple', width=0.003 )
                
        plt.title(title)
        plt.xlabel("x (metres)")
        plt.ylabel("y (metres)")
        if save == True:
            fig.savefig("plots/"+filepath+file_name + self.time+".svg", format="svg")
            fig.savefig("plots/"+filepath+file_name + self.time+".png", format="png")
            plt.close()
        else:
            plt.show()    
            

    def plot_end_effector_motion_x_y(self, link_lengths ,x_y_coordinates, s_axisq1, save=True, marker_size=5, file_name="workspace.png"):
        """
        produce a plot of qhow the robot moves in the workspace
        """           
        x_val = [x[0] for x in x_y_coordinates]
        y_val = [x[1] for x in x_y_coordinates]
        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
        plt.plot(x_val[0],y_val[0],'or', color='red', ms=marker_size, label="first end effector position")
        plt.plot(x_val[1:len(x_val)-1],y_val[1:len(y_val)-1],'or', color='orange', ms=5, label="end effector")
        plt.plot(x_val[-1:],y_val[-1:],'or', color='blue', ms=marker_size, label="last effector position")
            
        plt.grid(color='black', linestyle='-', linewidth=0.5)
        plt.legend()
        
        
        #scale axis
        max_extension = link_lengths[0] + link_lengths[1] + 0.2
        plt.xlim(right=max_extension) #xmax is your value
        plt.xlim(left=-max_extension) #xmin is your value
        plt.ylim(top=max_extension) #ymax is your value
        plt.ylim(bottom=-max_extension) #ymin is your value
        
        plt.title("Workspace_trajectory")
        plt.xlabel("x (metres)")
        plt.ylabel("y (metres)")
        if save == True:
            fig.savefig("plots/end_effector_motion/"+file_name + self.time)
            plt.close()
        else:
            plt.show()      


    def plot_each_motion_stage_x_y(self, link_lengths,x_y_coordinates, s_axisq1, save_path="plots/gifs/robot_motion_construction_images/", save=True, marker_size=5):
        """
        produce and save a plot for each part of the robot motion in the workspace
        this will allow for animated gif to be produced of the motion
        """
        x_val = [x[0] for x in x_y_coordinates]
        y_val = [x[1] for x in x_y_coordinates]
        
        q1 = [x[1] for x in s_axisq1]
        
       
        i = 0
        for x in x_val:
            
            if save == True:
                fig = plt.figure(dpi=600)
            else:
                fig = plt.figure()
            plt.plot(x_val[i],y_val[i],'or', color='orange', ms=marker_size, label="end effector")
            
            #attach labels on the first loop
            plt.plot([link_lengths[0]*m.cos(q1[i]), x],\
                      [link_lengths[0]*m.sin(q1[i]), y_val[i]], '-', color='g', label="links")
            
            plt.plot([0, link_lengths[0]*m.cos(q1[i])],\
                      [0, link_lengths[0]*m.sin(q1[i])], '-', color='g')
            
            plt.plot(link_lengths[0]*m.cos(q1[i]),\
                      link_lengths[0]*m.sin(q1[i]), 'or', color='b', ms=2, label="joint")

        
            
            plt.grid(color='black', linestyle='-', linewidth=0.5)
            plt.legend()

    
            plt.title("Workspace_trajectory")
            plt.xlabel("x (metres)")
            plt.ylabel("y (metres)")    

            #scale axis
            max_extension = link_lengths[0] + link_lengths[1] + 0.2
            plt.xlim(right=max_extension) #xmax is your value
            plt.xlim(left=-max_extension) #xmin is your value
            plt.ylim(top=max_extension) #ymax is your value
            plt.ylim(bottom=-max_extension) #ymin is your value 
            
           
            if save == True:
                fig.savefig(save_path+str(i))
                plt.close()
            else:
                plt.show()   

            i = i + 1

    def sub_q1_q2_v_robot_motion(self, coordinates_q1_q2, x_y_coordinates, save=True, marker_size = 5 ,file_name="sub plot"):
        """
        This method will produce a subplot array containing q1vq2 and the motion of the robot in the workspace
        """
        pass
    
    def plot_bound_vectors(self, evaluated_bounds, save=True, marker_size=1, filepath="bound_plot"):

        s = [x[0] for x in evaluated_bounds]
        sd = [x[1] for x in evaluated_bounds]
        L = [x[2] for x in evaluated_bounds]
        U = [x[3] for x in evaluated_bounds]
        
        s = np.array(s, dtype=np.float32) 
        sd = np.array(sd, dtype=np.float32)
        L = np.array(L, dtype=np.float32)
        U = np.array(U, dtype=np.float32)

        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
        ax1 = plt.subplot2grid((1,1),(0,0))
        #print(sd[1], U[1])
        ax1.plot(s,sd,'or',ms=marker_size)
        plt.xlabel("s")
        plt.ylabel("$\dot{s}$")
                
        plt.quiver(s, sd, sd, U)
        
        plt.xlim(right=1) #xmax is your value
        plt.xlim(left=0) #xmin is your value

        if save == True:
            fig.savefig("plots/"+filepath + dt.datetime.now().strftime('_%Y_%m_%d_%H_%M_%S'))
            plt.close()
        else:
            plt.show()

    def plot_simulation_parameters(self, s_axisq1, s_axisq2, coordinates_q1_q2, link_lengths, x_y_coordinates, \
                                     plot_q1_against_s=[0, "q1 v s"],\
                                     plot_q2_against_s=[0, "q2 v s"],\
                                     plot_q1_against_q2=[0, "q1, vs q2"],\
                                     plot_motion_x_y=[0, "workspace motion"],\
                                     plot_end_effector_motion=[0, "end effector motion"],\
                                     make_robot_motion_gif=[0, 50, "robot_motion"],\
                                     sub_plot_plot_q1_against_q2_and_motion_x_y =[0, "sub plots q1vq2, workspace"],\
                                     save=False):
        """
        method to generate many plots
        """
       
        if plot_q1_against_s[0] == 1:
            self.plot_q1_against_s(s_axisq1, save, 5, plot_q1_against_s[1])
        
        if plot_q2_against_s[0] == 1:
            self.plot_q2_against_s(s_axisq2, save, 5, plot_q2_against_s[1])
        
        if plot_q1_against_q2[0] == 1:
            self.plot_q1_against_q2(coordinates_q1_q2, save, 5, plot_q1_against_q2[1])
        
        if plot_motion_x_y[0] == 1:
            self.plot_robot_motion_x_y(link_lengths, x_y_coordinates, s_axisq1 ,save, 5,plot_motion_x_y[1])
        
        if plot_end_effector_motion[0] ==1:
            self.plot_end_effector_motion_x_y(link_lengths, x_y_coordinates, s_axisq1, save, 5 , plot_end_effector_motion[1])
        
        if make_robot_motion_gif[0] == 1:
            #just testing out making a gif
            path = "plots/gifs/robot_motion_construction_images/"
            self.plot_each_motion_stage_x_y(link_lengths, x_y_coordinates, s_axisq1, path ,save, 5)
                        
            path_backslashes = "plots\\gifs\\robot_motion_construction_images\\"
            save_offset = "plots\\gifs\\"   
            filename = make_robot_motion_gif[2]
            frame_duration = make_robot_motion_gif[1]
            png_to_gif.compile_gif(path_backslashes, save_offset, filename, frame_duration, self.time)
         
        if sub_plot_plot_q1_against_q2_and_motion_x_y[0] == 1:
            self.sub_q1_q2_v_robot_motion(coordinates_q1_q2, x_y_coordinates, save, 5,sub_plot_plot_q1_against_q2_and_motion_x_y[1])

    def plot_potential_collision_energies_s(self, potential_collision_energy, save=True,  marker_size=1, filepath="Ek_vs_s"):
        """
        Method to produce a plot of s vs potential energy dissapated in a collision
        """
       
        x_val = [x[0] for x in potential_collision_energy]
        y_val = [y[2] for y in potential_collision_energy]
        
        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
            
        ax1 = plt.subplot2grid((1,1),(0,0))
        
        ax1.plot(x_val, y_val,'or',ms=marker_size)
        plt.grid(color='black', linestyle='-', linewidth=0.5)
        plt.xlabel("s")
        plt.ylabel("$E_k$")
        if save == True:
            fig.savefig("plots/"+ "Ek_vs_s/" +filepath + self.time)
            plt.close()
        else:
            plt.show()
        
    def plot_potential_collision_energies_sdot(self, potential_collision_energy, save=True,  marker_size=1, filepath="Ek_vs_sdot"):
        """
        Method to produce a plot of s vs potential energy dissapated in a collision
                Arguments:
            potential_collision_energy = [(s1, sd1, Ek1),...(sn, sdn, Ekn)]
        
        """
       
        x_val = [x[1] for x in potential_collision_energy]
        y_val = [y[2] for y in potential_collision_energy]
        
        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
            
        ax1 = plt.subplot2grid((1,1),(0,0))
        
        ax1.plot(x_val, y_val,'or',ms=marker_size)
        plt.grid(color='black', linestyle='-', linewidth=0.5)
        plt.xlabel("$\dot{s}$")
        plt.ylabel("$E_k$")
        if save == True:
            fig.savefig("plots/"+ "Ek_vs_sdot/" +filepath + self.time)
            plt.close()
        else:
            plt.show()
            
            

    def plot_potential_collision_energies_s_sdot(self, potential_collision_energy, save=True,  marker_size=1, filepath="Ek_vs_s_sdot"):
        """
        Method to produce a plot of s vs potential energy dissapated in a collision
        Arguments:
            potential_collision_energy = [(s1, sd1, Ek1),...(sn, sdn, Ekn)]
        """
       
        x_val = [x[0] for x in potential_collision_energy]
        y_val = [x[1] for x in potential_collision_energy]
        z_val = [y[2] for y in potential_collision_energy]
        
         
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
            
        ax1 =  fig.add_subplot(111, projection='3d')
        
        ax1.plot(x_val, y_val, z_val) 
        
        #ax1.plot(x_val, y_val,'or',ms=marker_size)
        #plt.grid(color='black', linestyle='-', linewidth=0.5)
        plt.xlabel("s")
        plt.ylabel("$\dot{s}$")
        ax1.set_zlabel("$E_k$")
        if save == True:
            fig.savefig("plots/"+ "Ek_vs_s_sdot/" +filepath + self.time)
            plt.close()
        else:
            plt.show()

    def plot_potential_collision_energies_s_sdot_surf(self, potential_collision_energy, save=True,  marker_size=1, filepath="Ek_vs_s_sdot_contour"):
        """
        function to take in data of the common format of this toolbox and convert it to produce surface plots
        on matplotlib
        Arguments:
            potential_collision_energy = [(s_1, sd_1, Ek1),... , 
                                          (s_n, sd_n, Ekn)]
        return:
        """
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()

        ax =  fig.add_subplot(111, projection='3d')
  
        x_val = [x[0] for x in potential_collision_energy]
        y_val = [x[1] for x in potential_collision_energy]
        z_val = [y[2] for y in potential_collision_energy]
        
        ax.plot(x_val, y_val, z_val) 
        
        plt.grid(color='black', linestyle='-', linewidth=0.5)
        plt.xlabel("s")
        plt.ylabel("$\dot{s}$")
        ax.set_zlabel("$E_k$")
        
        plt.xticks(np.arange(min(x_val), max(x_val)+0.25, 0.25))
        plt.yticks(np.arange(min(y_val), max(y_val)+5, 5))
        
        if save == True:
            fig.savefig("plots/"+ "Ek_vs_s_sdot_surface/" +filepath + self.time)
            plt.close()
        else:
            plt.show()
        
    def generate_collision_energy_angle_plot(self, angle_vs_energy,  save=True, marker_size=1, filepath="angle_vs_max_energy"):
        """
        this function simply takes in a list of tuples and plots them
        Arguments:
            admissable_region [(s1,sd1), (s2,sd2), ... ,(sn,sdn)]
            
        produces a plot of the s sdot region
        """
        angle = [m.degrees(x[2]) for x in angle_vs_energy]
        max_energy = [x[3] for x in angle_vs_energy]
        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
            
        ax1 = plt.subplot2grid((1,1),(0,0))
        
        ax1.plot(angle, max_energy,'or',ms=marker_size)
        plt.grid(color='black', linestyle='-', linewidth=0.5)
        plt.xlabel("angle / degrees")
        plt.ylabel("$E_{kmax}$")
        plt.xticks(np.arange(min(angle), max(angle)+60, 60))        
        if save == True:
            fig.savefig("plots/angle_vs_max_energy/"+filepath + self.time)
            plt.close()
        else:
            plt.show()
    
    
    def plot_max_collision_energy_vs_direction_s(self, s_angle_energy, save=True,  marker_size=1, filename="Ek_vs_s_sdot"):
        """
        Method to produce a plot of s vs potential energy dissapated in a collision
        Arguments:
            potential_collision_energy = [(s1, sd1, Ek1),...(sn, sdn, Ekn)]
        """
       
        x_val = [x[0] for x in s_angle_energy]
        y_val = [x[2] for x in s_angle_energy]
        z_val = [y[3] for y in s_angle_energy]
        
        y_val = [m.degrees(x) for x in y_val]
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
            
        ax1 =  fig.add_subplot(111, projection='3d')
        
        ax1.plot(x_val, y_val, z_val) 
        
        #ax1.plot(x_val, y_val,'or',ms=marker_size)
        #plt.grid(color='black', linestyle='-', linewidth=0.5)
        plt.xlabel("s")
        plt.ylabel("Angle (degrees)")
        ax1.set_zlabel("Collision_energy")
        if save == True:
            fig.savefig("plots/"+ "max_collision_energy_direction_s/" +filename + self.time)
            plt.close()
        else:
            plt.show()
            
    def generate_state_space_plot_with_colour_markers_for_acceleration(self, admissible_regions,\
                                                                       save=False, marker_size=3, \
                                                                       filename="admissible_plot_acceleration_regions", \
                                                                        filepath="admissible_plot_acceleration_regions/"):
        """        
        Parameters
        ----------
        admissible_regions : list of lists of tuples
            list of 4 different regions to plot in sperate colours
        save : bool, optional
            DESCRIPTION. save the plot or not.
        marker_size : size to plot points, optional
            DESCRIPTION. The default is 1.
        filename : TYPE, optional
            DESCRIPTION. The default is "admissible_plot_accelation_regions".
        filepath : TYPE, optional
            DESCRIPTION. The default is "admissible_plot_accelation_regions/".

        Returns
        -------
        None.

        """
        region_1 = admissible_regions[0]
        region_2 = admissible_regions[1]  
        region_3 = admissible_regions[2]  
        region_4 = admissible_regions[3]  
                
        x1 = [x[0] for x in region_1]
        y1 = [x[1] for x in region_1]
                
        x2 = [x[0] for x in region_2]
        y2 = [x[1] for x in region_2]
        
        x3 = [x[0] for x in region_3]
        y3 = [x[1] for x in region_3]
        
        x4 = [x[0] for x in region_4]
        y4 = [x[1] for x in region_4]
        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
            
        ax1 = plt.subplot2grid((1,1),(0,0))
        
        reg_1, = ax1.plot(x1,y1, 'or',ms=marker_size, color='r')
        reg_1.set_label("U1, L1 bounds region")
        
        reg_2, = ax1.plot(x2,y2, 'or',ms=marker_size, color='g')
        reg_2.set_label("U1, L2 bounds region")
        
        reg_3, = ax1.plot(x3,y3, 'or',ms=marker_size, color='b')
        reg_3.set_label("U2, L1 bounds region")
        
        reg_4, = ax1.plot(x4,y4, 'or',ms=marker_size, color='c')
        reg_4.set_label("U2, L2 bounds region")
        
        plt.legend()
        
        
        plt.xlabel("s")
        plt.ylabel("$\dot{s}$")
        
        if save == True:
            fig.savefig("plots/" +filepath + filename + self.time)
            plt.close()
        else:
            plt.show()        

    def s_sdot_sddo_limit_plot(self, admissible_regions,\
                                save=False, marker_size=3, \
                                filename="admissible_plot_acceleration_regions_3D", \
                                 filepath="admissible_plot_acceleration_regions/"):
        """
        Parameters
        ----------
        admissible_regions : list of lists of tuples
            list of 4 different regions to plot in sperate colours with the maximum and minimum sdd plots plotted on z axis
        save : bool, optional
            DESCRIPTION. save the plot or not.
        marker_size : size to plot points, optional
            DESCRIPTION. The default is 1.
        filename : TYPE, optional
            DESCRIPTION. The default is "admissible_plot_accelation_regions".
        filepath : TYPE, optional
            DESCRIPTION. The default is "admissible_plot_accelation_regions/".
        Returns
        -------
        None.
        """
        region_1 = admissible_regions[0]
        region_2 = admissible_regions[1]  
        region_3 = admissible_regions[2]  
        region_4 = admissible_regions[3]  
        print(len(region_1), len(region_2) , len(region_3) ,len(region_4))
        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
        #ax1 = plt.subplot2grid((1,1),(0,0))
        ax1 =  fig.add_subplot(111, projection='3d')
        
        """
        x1 = [x[0] for x in region_1]
        y1 = [x[1] for x in region_1]
        z1_l = [x[2] for x in region_1]
        z1_u = [x[3] for x in region_1]
        reg_1, = ax1.plot(x1,y1,z1_l, 'or',ms=marker_size, color='r')
        ax1.plot(x1,y1,z1_u, 'or',ms=marker_size, color='r')
        reg_1.set_label("U1, L1 bounds region")
        """       
        
        x2 = [x[0] for x in region_2]
        y2 = [x[1] for x in region_2]
        z2_l = [x[2] for x in region_2]
        z2_u = [x[3] for x in region_2]
        reg_2, = ax1.plot(x2,y2,z2_l, 'or',ms=marker_size, color='g')
        ax1.plot(x2,y2,z2_u, 'or',ms=marker_size, color='g')
        reg_2.set_label("U1, L2 bounds region")    
    
        """   
        x3 = [x[0] for x in region_3]
        y3 = [x[1] for x in region_3]
        z3_l = [x[2] for x in region_3]
        z3_u = [x[3] for x in region_3]
        reg_3, = ax1.plot(x3, y3, z3_l, 'or',ms=marker_size, color='b')
        ax1.plot(x3, y3, z3_u, 'or',ms=marker_size, color='b')
        reg_3.set_label("U2, L1 bounds region")

        x4 = [x[0] for x in region_4]
        y4 = [x[1] for x in region_4]
        z4_l = [x[2] for x in region_4]
        z4_u = [x[3] for x in region_4]
        reg_4, = ax1.plot(x4,y4,z4_l, 'or',ms=marker_size, color='c')
        ax1.plot(x4,y4,z4_u, 'or',ms=marker_size, color='c')
        reg_4.set_label("U2, L2 bounds region")
    
        """

        plt.legend()
        
        plt.xlabel("s")
        plt.ylabel("$\dot{s}$")
        ax1.set_zlabel("$\ddot{s}$")
        
        print("near done")
        if save == True:
            fig.savefig("plots/" +filepath + filename + self.time)
            plt.close()
        else:
            plt.show()            
        print("plot should be here #")
        
                
    def overlay_trajectories_with_admissible_region(self, admissible_region,\
                                                    trajectory_list, label_list,\
                                                    color_list,\
                                                    save=True, marker_size=1,\
                                                    filepath= "overlayed_plots/",\
                                                    filename= "overlayed_trajectories",\
                                                    title= "default title"):
        """
        Parameters
        ----------
        admissible_region : list of states [(x1,y1),...,(xn,yn) ] or "N/A"
            list of states that make up the admissible region to plot
        trajectory_list : list of trajectorites [trajectory_1, ...,trajectory_N]
            list of trajectories to plot
        label_list : list of labels to go with each of the trajectories [lab1, lab2,... , labN]
            labels should be strings
        color_list : list of labels to go with each of the trajectories [col1, col2,... , colN]
            colors should be colors or the string representing a color
        save : Bool, optional
            save produced plots. The default is True.
        marker_size : size of markers, float, optional
            DESCRIPTION. The default is 1.
        filepath : string, optional
            DESCRIPTION. The default is "overlayed_plots/".
        filename : string, optional
            DESCRIPTION. The default is "overlayed_trajectories".

        Returns
        -------
        None.

        """

        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
        
        #if statement to 
        if admissible_region == "N/A":
            x1 = [x[0] for x in trajectory_list[0]]
            x2 = [x[1] for x in trajectory_list[0]]
            things_to_plot = [(x1, x2, label_list[0], color_list[0])]
            i=1
        else:
            x1_admissible = [x[0] for x in admissible_region]
            x2_admissible = [x[1] for x in admissible_region]    
            things_to_plot = [(x1_admissible, x2_admissible, "$X$", 'r')]
            i=0
        
        number_of_trajectories = len(trajectory_list)
        #count through trajectories
        while i < number_of_trajectories:
            x1 = [x[0] for x in trajectory_list[i]]
            x2 = [x[1] for x in trajectory_list[i]]
            things_to_plot.append((x1,x2, label_list[i], color_list[i]))
            i = i + 1
        
        i = 1
        for thing in things_to_plot:
            x = np.array(thing[0], dtype=np.float32)
            y = np.array(thing[1], dtype=np.float32)
            plt.plot(x, y, ms=marker_size, color=thing[3], label=thing[2])    
            i =  i + 1
            
        plt.grid()
        plt.xlabel("$x_1$")
        plt.ylabel("$x_2$")
        plt.xticks(np.arange(0, 1.1, step=0.1))    
        plt.title(title)
        if save == True:
            fig.savefig("plots/" + filepath + filename + self.time + ".jpg", format="jpg")
            fig.savefig("plots/" + filepath + filename + self.time + ".svg", format="svg")
            plt.close()
        else:
            plt.show()          

    def paper_plot_constraint_reach_avoid_target(self, target_set, trajectory_list,label_list,\
                                                    color_list,\
                                                    save=True, marker_size=1,\
                                                    filepath= "paper/",\
                                                    filename= "sets",\
                                                    title= "default title"):
        """
        Parameters
        ----------
        admissible_region : list of states [(x1,y1),...,(xn,yn) ] or "N/A"
            list of states that make up the admissible region to plot
        trajectory_list : list of trajectorites [trajectory_1, ...,trajectory_N]
            list of trajectories to plot
        label_list : list of labels to go with each of the trajectories [lab1, lab2,... , labN]
            labels should be strings
        color_list : list of labels to go with each of the trajectories [col1, col2,... , colN]
            colors should be colors or the string representing a color
        save : Bool, optional
            save produced plots. The default is True.
        marker_size : size of markers, float, optional
            DESCRIPTION. The default is 1.
        filepath : string, optional
            DESCRIPTION. The default is "overlayed_plots/".
        filename : string, optional
            DESCRIPTION. The default is "overlayed_trajectories".

        Returns
        -------
        None.

        """
        
        if save == True:
            fig, ax = plt.subplots(dpi=600)
            #fig = plt.figure(dpi=600)
        else:
            fig, ax = plt.subplots()
        
        #if statement to 
        print(trajectory_list[0])
        
        x1 = [x[0] for x in trajectory_list[0]]
        x2 = [x[1] for x in trajectory_list[0]]
        things_to_plot = [(x1, x2, label_list[0], color_list[0])]
        i=1
        
        number_of_trajectories = len(trajectory_list)
        #count through trajectories
        while i < number_of_trajectories:
            x1 = [x[0] for x in trajectory_list[i]]
            x2 = [x[1] for x in trajectory_list[i]]
            things_to_plot.append((x1,x2, label_list[i], color_list[i]))
            i = i + 1
        
        i = 1
        for thing in things_to_plot:
            x = np.array(thing[0], dtype=np.float32)
            y = np.array(thing[1], dtype=np.float32)
            #print(x)
            plt.plot(x, y, ms=marker_size, color=thing[3], label=thing[2])
            #plt.plot(0.9, 3, ms=5, color='red', label=thing[2])  
            i =  i + 1
        

        #print(x2)
        plt.plot(target_set[0], target_set[1], ms=5, color='red', label=thing[2])   
        

        custom_lines = [Line2D([0], [0], color='darkgreen', lw=4),\
                        Line2D([0], [0], color='orange', lw=4),\
                        Line2D([0], [0], color='red', lw=4)]
            
        ax.legend(custom_lines, ['$Bd(\mathcal{X})$', '$Bd(\mathcal{R}(x^{*}))$', '$x^*$'],  prop={'size': 12})

        plt.grid()
        plt.xlabel("$x_1$")
        plt.ylabel("$x_2$")
        plt.xticks(np.arange(0, 1.1, step=0.1))    
        plt.title(title)
        if save == True:
            fig.savefig("plots/" + filepath + filename + ".jpg", format="jpg")
            fig.savefig("plots/" + filepath + filename + ".svg", format="svg")
            plt.close()
        else:
            plt.show()   
            
            
    def overlay_trajectories_with_admissible_region_with_guides(self, admissible_region,\
                                                    trajectory_list, \
                                                    upper_guide,\
                                                    lower_guide,\
                                                    label_list,\
                                                    color_list,\
                                                    save=True, marker_size=1,\
                                                    filepath= "overlayed_plots/",\
                                                    filename= "overlayed_trajectories",\
                                                    title= "default title"):
        """
        Parameters
        ----------
        admissible_region : list of states [(x1,y1),...,(xn,yn) ] or "N/A"
            list of states that make up the admissible region to plot
        trajectory_list : list of trajectorites [trajectory_1, ...,trajectory_N]
            list of trajectories to plot
        label_list : list of labels to go with each of the trajectories [lab1, lab2,... , labN]
            labels should be strings
        color_list : list of labels to go with each of the trajectories [col1, col2,... , colN]
            colors should be colors or the string representing a color
        save : Bool, optional
            save produced plots. The default is True.
        marker_size : size of markers, float, optional
            DESCRIPTION. The default is 1.
        filepath : string, optional
            DESCRIPTION. The default is "overlayed_plots/".
        filename : string, optional
            DESCRIPTION. The default is "overlayed_trajectories".

        Returns
        -------
        None.

        """

        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
        
        #if statement to 
        if admissible_region == "N/A":
            x1 = [x[0] for x in trajectory_list[0]]
            x2 = [x[1] for x in trajectory_list[0]]
            things_to_plot = [(x1, x2, label_list[0], color_list[0])]
            i=1
        else:
            x1_admissible = [x[0] for x in admissible_region]
            x2_admissible = [x[1] for x in admissible_region]    
            things_to_plot = [(x1_admissible, x2_admissible, "$X$", 'r')]
            i=0
        
        number_of_trajectories = len(trajectory_list)
        #count through trajectories
        while i < number_of_trajectories:
            x1 = [x[0] for x in trajectory_list[i]]
            x2 = [x[1] for x in trajectory_list[i]]
            things_to_plot.append((x1,x2, label_list[i], color_list[i]))
            i = i + 1
        
        i = 1
        for thing in things_to_plot:
            x = np.array(thing[0], dtype=np.float32)
            y = np.array(thing[1], dtype=np.float32)
            plt.plot(x, y, ms=3*marker_size, color=thing[3], label=thing[2])    
            i =  i + 1
            
        
        plt.plot(np.linspace(0,1,len(upper_guide)), upper_guide, ms=marker_size, color='red', label='upper_guide')  
        plt.plot(np.linspace(0,1,len(lower_guide)), lower_guide, ms=marker_size, color='blue', label='lower_guide')  
        
        plt.grid()
        plt.xlabel("$x_1$")
        plt.ylabel("$x_2$")
        plt.xticks(np.arange(0, 1.1, step=0.1))    
        plt.title(title)
        if save == True:
            fig.savefig("plots/" + filepath + filename + self.time + ".png", format="png")
            fig.savefig("plots/" + filepath + filename + self.time + ".svg", format="svg")
            plt.close()
        else:
            plt.show()          


    def plot_limit_curves_paper(self, VLC, soft_constraints,\
                                save=False, marker_size=1,\
                                filepath= "paper_plots/",\
                                filename= "phase_plane_constraint_plots",\
                                title= "constraints in x_1 x_2 region"):


        #print(VLC)
        x1 = [x[0] for x in VLC]
        y1 = [x[1] for x in VLC]

        x2 = [x[0] for x in soft_constraints]
        y2 = [x[1] for x in soft_constraints]

        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
        
        plt.plot(x1, y1, ms=1,\
                 color='green',\
                 label="VLC")
            
        plt.plot(x2, y2, ms=1,\
                 color='black',\
                 label="ACC",\
                 linestyle="dashed")                

        try:
            labelLines(plt.gca().get_lines(), align=False, fontsize=10)
        except:
            print("error putting labels on lines resorting to legend")
            plt.legend( bbox_to_anchor=(1.01, 1), loc='upper left', fontsize='normal')
                        
        plt.xlabel("$x_1$")
        plt.ylabel("$x_2$")
        plt.xticks(np.arange(0, 1.1, step=0.1))            
        ax = plt.gca()
        ax.set(xlim=(0, 1), ylim=(0, 22))
        
        if save == True:
            fig.savefig("plots/" + filepath + filename + self.time + ".svg", format="svg")
            fig.savefig("plots/" + filepath + filename + self.time + ".jpg", format="jpg")
            plt.close()
        else:
            plt.show()
                
    def plot_list(self, list_to_plot, save=False,filepath="paper_plots/",\
                        filename="actuation_level_8_1", title="list of things"):
        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
        
        x = np.arange(1, len(list_to_plot)+1, 1)
        y = list_to_plot

        plt.plot(x, y, 'or', ms=1,color='red')
            
        plt.xlabel("$N$")
        plt.ylabel("$\lambda$") 
        plt.title("Actuation level plot")
        #ax.set(ylim=(0, 1))
        if save == True:
            fig.savefig("plots/" + filepath + filename + self.time + ".svg", format="svg")
            fig.savefig("plots/" + filepath + filename + self.time + ".jpg", format="jpg")
            plt.close()
        else:
            plt.show()            
            
            
    def plot_actuation_level_list(self, list_to_plot, save=False,filepath="paper_plots/",\
                        filename="actuation_level_8_1", title="list of things"):
        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
        
        x = np.arange(1, len(list_to_plot)+1, 1)
        y = list_to_plot

        plt.plot(x, y, 'or', ms=1,color='red')
            
        plt.xlabel("$N$")
        plt.ylabel("$\lambda$") 
        plt.title(title)
        #ax.set(ylim=(0, 1))
        if save == True:
            fig.savefig("plots/" + filepath + filename + self.time + ".svg", format="svg")
            fig.savefig("plots/" + filepath + filename + self.time + ".jpg", format="jpg")
            plt.close()
        else:
            plt.show()    
            
    def plot_input_list(self, list_to_plot, save=False,filepath="paper_plots/",\
                        filename="actuation_level_8_1", title="list of things"):
        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()
        
        x = np.arange(1, len(list_to_plot)+1, 1)
        y = list_to_plot

        plt.plot(x, y, 'or', ms=1,color='red')
            
        plt.xlabel("$N$")
        plt.ylabel("$input$") 
        plt.title(title)
 
        #ax.set(ylim=(0, 1))
        if save == True:
            fig.savefig("plots/" + filepath + filename + self.time + ".svg", format="svg")
            fig.savefig("plots/" + filepath + filename + self.time + ".jpg", format="jpg")
            plt.close()
        else:
            plt.show()                
    
    def plot_actuation_level_vs_x(self, list_to_plot, save=False,filepath="paper_plots/",\
                        filename="actuation_level_vs_x1_8_1", title="list of things", axis_label="$\lambda$"):
        
        print("uwbufweb")#, list_to_plot)
        
        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()      
        
        actuation_level = [x[0] for x in list_to_plot]
        x1 = [x[1][0] for x in list_to_plot]
        x2 = [x[1][1] for x in list_to_plot]
    
    
    
    
        print(len(actuation_level), len(x1), len(x2))
        plt.plot(x1, actuation_level, 'or', ms=1,color='red')
        plt.grid()
        plt.xlabel("$x_1$")
        plt.ylabel(axis_label) 
        plt.title(title)
        plt.xticks(np.arange(0, 1.1, step=0.1))  
        #ax.set(ylim=(0, 1))
        if save == True:
            fig.savefig("plots/" + filepath + filename + self.time + ".svg", format="svg")
            fig.savefig("plots/" + filepath + filename + self.time + ".jpg", format="jpg")
            plt.close()
        else:
            plt.show()  
    
    
    
    def plot_two_vs_x1(self, list_to_plot, save=False,filepath="paper_plots/",\
                        filename="two_list_plot_vs_x1_8_1", title="list of things", axis_label="$quantity$"):
        
        print("uwbufweb")#, list_to_plot)
        
        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()      
        
        list1 = [x[0] for x in list_to_plot]
        x1 = [x[2][0] for x in list_to_plot]
        x2 = [x[2][1] for x in list_to_plot]
    
    
        list2 =  [x[1] for x in list_to_plot]    
        x12 = [x[2][0] for x in list_to_plot]
        x22 = [x[2][1] for x in list_to_plot]
    
    
        #print(len(list1), len(x1), len(x2))
        
        plt.plot(x1, list1, 'or', ms=1,color='red')
        plt.plot(x1, list2, 'or', ms=1,color='blue')
        plt.grid()
        plt.xlabel("$x_1$")
        plt.ylabel(axis_label) 
        plt.title(title)
        plt.xticks(np.arange(0, 1.1, step=0.1))  
        #ax.set(ylim=(0, 1))
        if save == True:
            fig.savefig("plots/" + filepath + filename + self.time + ".svg", format="svg")
            fig.savefig("plots/" + filepath + filename + self.time + ".jpg", format="jpg")
            plt.close()
        else:
            plt.show()    
    
    
    def plot_guides_with_control(self, control_trajectory,\
                                        upper_guide,\
                                        lower_guide,\
                                        label_list,\
                                        color_list,\
                                        save,\
                                        marker_size = 1,\
                                        filepath="paper_plots/",\
                                        filename="controller_plot",\
                                        title="controller"):
        
        """
        Parameters
        ----------
        control_trajectory : list of states [(x1,y1),...,(xn,yn) ] or "N/A"
            list of states that make up the control trajectory to plot
        upper_guide : list of states [(x1,y1),...,(xn,yn) ] or "N/A"
            list of states that make up the upper guide to plot
        lower_guide : list of states [(x1,y1),...,(xn,yn) ] or "N/A"
            list of states that make up the lower guide to plot    
        label_list : list of labels to go with each of the trajectories [lab1, lab2,... , labN]
            labels should be strings
        color_list : list of labels to go with each of the trajectories [col1, col2,... , colN]
            colors should be colors or the string representing a color
        save : Bool, optional
            save produced plots. The default is True.
        marker_size : size of markers, float, optional
            DESCRIPTION. The default is 1.
        filepath : string, optional
            DESCRIPTION. The default is "overlayed_plots/".
        filename : string, optional
            DESCRIPTION. The default is "overlayed_trajectories".

        Returns
        -------
        None.

        """

        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()

        if control_trajectory == "N/A":
            pass
        else:
            x1 = [x[0] for x in control_trajectory]
            x2 = [x[1] for x in control_trajectory]
            plt.plot(x1, x2, ms=3*marker_size, color=color_list[2], label=label_list[2])                
            
            
        if upper_guide == "N/A":
            pass
        else:
            x1 = [x[0] for x in upper_guide]
            x2 = [x[1] for x in upper_guide]
            plt.plot(x1, x2, ms=3*marker_size, color=color_list[0], label=label_list[0])                            
        
        if lower_guide == "N/A":
            pass
        else:
            x1 = [x[0] for x in lower_guide]
            x2 = [x[1] for x in lower_guide]
            plt.plot(x1, x2, ms=3*marker_size, color=color_list[1], label=label_list[1])                
        

        plt.grid()
        plt.xlabel("$x_1$")
        plt.ylabel("$x_2$")
        plt.xticks(np.arange(0, 1.1, step=0.1))    
        plt.title(title)

        if save == True:
            fig.savefig("plots/" + filepath + filename + ".png", format="png")
            fig.savefig("plots/" + filepath + filename + ".svg", format="svg")
            plt.close()
        else:
            plt.show()          

    def plot_guides_with_control_and_admissible(self, control_trajectory,\
                                        upper_guide,\
                                        lower_guide,\
                                        upper_admissible,\
                                        lower_admissible,\
                                        save,\
                                        marker_size = 1,\
                                        filepath="paper_plots/",\
                                        filename="controller_plot",\
                                        title="controller"):
        
        """
        Parameters
        ----------
        control_trajectory : list of states [(x1,y1),...,(xn,yn) ] or "N/A"
            list of states that make up the control trajectory to plot
        upper_guide : list of states [(x1,y1),...,(xn,yn) ] or "N/A"
            list of states that make up the upper guide to plot
        lower_guide : list of states [(x1,y1),...,(xn,yn) ] or "N/A"
            list of states that make up the lower guide to plot    
        label_list : list of labels to go with each of the trajectories [lab1, lab2,... , labN]
            labels should be strings
        color_list : list of labels to go with each of the trajectories [col1, col2,... , colN]
            colors should be colors or the string representing a color
        save : Bool, optional
            save produced plots. The default is True.
        marker_size : size of markers, float, optional
            DESCRIPTION. The default is 1.
        filepath : string, optional
            DESCRIPTION. The default is "overlayed_plots/".
        filename : string, optional
            DESCRIPTION. The default is "overlayed_trajectories".

        Returns
        -------
        None.

        """

        
        if save == True:
            fig = plt.figure(dpi=600)
        else:
            fig = plt.figure()

        if control_trajectory == "N/A":
            pass
        else:
            x1 = [x[0] for x in control_trajectory]
            x2 = [x[1] for x in control_trajectory]
            plt.plot(x1, x2, ms=3*marker_size, color='purple')                
            
            
        if upper_guide == "N/A":
            pass
        else:
            x1 = [x[0] for x in upper_guide]
            x2 = [x[1] for x in upper_guide]
            plt.plot(x1, x2, ms=3*marker_size, color='blue')                            
        
        if lower_guide == "N/A":
            pass
        else:
            x1 = [x[0] for x in lower_guide]
            x2 = [x[1] for x in lower_guide]
            plt.plot(x1, x2, ms=3*marker_size, color='blue')                
        
        if upper_admissible == "N/A":
            pass
        else:
            x1 = [x[0] for x in upper_admissible]
            x2 = [x[1] for x in upper_admissible]
            plt.plot(x1, x2, ms=3*marker_size, color='orange')                
        
        if lower_admissible == "N/A":
            pass
        else:
            x1 = [x[0] for x in lower_admissible]
            x2 = [x[1] for x in lower_admissible]
            plt.plot(x1, x2, ms=3*marker_size, color='orange')                
        


        plt.grid()
        plt.xlabel("$x_1$")
        plt.ylabel("$x_2$")
        plt.xticks(np.arange(0, 1.1, step=0.1))    
        plt.title(title)

        if save == True:
            fig.savefig("plots/" + filepath + filename + ".png", format="png")
            fig.savefig("plots/" + filepath + filename + ".svg", format="svg")
            plt.close()
        else:
            plt.show()   
    
    
    
if __name__ == "__main__":             
    import numpy as np
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from scipy.spatial import cKDTree
    from scipy import interpolate
    
    fig = plt.figure()
    ax = fig.add_axes([0, 0, 1, 1], projection='3d')
    ax.axis('off')
    
    def upsample_coords(coord_list):
        # s is smoothness, set to zero
        # k is degree of the spline. setting to 1 for linear spline
        tck, u = interpolate.splprep(coord_list, k=1, s=0.0)
        upsampled_coords = interpolate.splev(np.linspace(0, 1, 100), tck)
        return upsampled_coords
    
    # target line
    x_targ = [1, 2, 3, 4, 5, 6, 7, 8]
    y_targ = [20, 100, 50, 120, 55, 240, 50, 25]
    z_targ = [20, 100, 50, 120, 55, 240, 50, 25]
    targ_upsampled = upsample_coords([x_targ, y_targ, z_targ])
    targ_coords = np.column_stack(targ_upsampled)
    
    # KD-tree for nearest neighbor search
    targ_kdtree = cKDTree(targ_coords)
    
    # line two
    x2 = [3.5,4.2,5.6,6.8,7.1,8.3,9.5]
    y2 = [25,35,14,67,88,44,120]
    z2 = [25,35,14,67,88,44,120]
    l2_upsampled = upsample_coords([x2, y2, z2])
    l2_coords = np.column_stack(l2_upsampled)
    
    # plot both lines
    ax.plot(x_targ, y_targ, z_targ, color='r', linewidth=1)
    ax.plot(x2, y2, z2, color='darkgreen', linewidth=1)
    
    # find intersections
    for i in range(len(l2_coords)):
        if i == 0:  # skip first, there is no previous point
            continue
    
        distance, close_index = targ_kdtree.query(l2_coords[i], distance_upper_bound=.5)
        # strangely, points infinitely far away are somehow within the upper bound
        if np.isinf(distance):
            continue
        
        # plot ground truth that was activated
        _x, _y, _z = targ_kdtree.data[close_index]
        ax.scatter(_x, _y, _z, 'gx')
        _x2, _y2, _z2 = l2_coords[i]
        ax.scatter(_x2, _y2, _z2, 'rx')  # Plot the cross point
    
    #print(_x,_y)
    plt.show()
    

        