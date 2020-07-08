# -*- coding: utf-8 -*-
"""
Created on Fri Apr  3 20:38:47 2020

@author: Ryan
This file contains a class that will be used to visualise different parts of the system
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d
import numpy as np
import math as m
import png_to_gif
import datetime as dt


class two_dof_robot_data_visualisation():
    
    def __init__(self, current_time):
        self.time = current_time
    
    def generate_state_space_plot(self, admissible_region,  save=True, marker_size=1, filepath="admissible_plot"):
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
        plt.xlabel("s")
        plt.ylabel("$\dot{s}$")
        if save == True:
            fig.savefig("plots/admissible_region/"+filepath + self.time)
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
            
    def plot_q1_against_q2(self, coordinates, save=True, marker_size = 5, file_name="q2 vs s.png"):
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
        plt.plot(x_val,y_val,'or',ms=marker_size)
        plt.grid(color='black', linestyle='-', linewidth=0.5)
        plt.title("Angle of q1 vs angle of q2")
        plt.xlabel("q1 (degrees)")
        plt.ylabel("q2 (degrees)")
        if save == True:
            fig.savefig("plots/q1_vs_q2/" + file_name + self.time)
            plt.close()
        else:
            plt.show()
        
        
    def plot_robot_motion_x_y(self, link_lengths, x_y_coordinates, s_axisq1, save=True, marker_size=5, file_name="robot_motion"):
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
        i = 0
        
        
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
            fig.savefig("plots/workspace_motion/"+file_name + self.time)
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
        
