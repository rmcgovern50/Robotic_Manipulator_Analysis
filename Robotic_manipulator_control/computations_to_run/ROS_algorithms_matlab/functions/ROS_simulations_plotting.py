# -*- coding: utf-8 -*-
"""
@author: Ryan
This file contains methods that produce plots
"""

from robot_data_visualisation import two_dof_robot_data_visualisation
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

def plot_multiple_trajectories(trajectory_list, current_time, folder, save=False ):
    
    plotter = two_dof_robot_data_visualisation(current_time)

    label_list = ['Td', 'Ta', 'T_constraint', 'upper_extreme','lower_exterme', 'Tlower', 'Tu1', 'Tu2', 'Tu3' ]
    color_list = ['orange', 'orange', 'darkgreen', 'orange', 'orange', 'orange', 'orange','orange', 'orange']

    T_len = len(trajectory_list)
    lab_len = len(label_list)
    col_len = len(color_list)
    #print("label list length ", lab_len)
    #print("color list length ", col_len)
    #print("trajectory list length ", T_len)
    
    if T_len > lab_len:
        diff = T_len - lab_len
        #print(diff)
        while diff > 0:
            label_list.append("T_extra")
            diff = T_len - len(label_list)
    
    if T_len > col_len:
        last_el = color_list[-1]
        diff = T_len - lab_len
        #print(diff)
        while diff >0:
            color_list.append(last_el)
            diff = T_len - len(color_list)            
   
    plotter.overlay_trajectories_with_admissible_region("N/A",\
                                                        trajectory_list,\
                                                        label_list,color_list,\
                                                        save,1,folder,\
                                                        "trajectory_plots", title="Region of Stablisability")

def plot_region_boundaries(target_interval, trajectory_list, current_time, folder, save=False, heading="Region of Stablisability" ):

    label_list = ['Td', 'Ta', 'T_constraint', 'upper_extreme','lower_exterme', 'Tlower', 'Tu1', 'Tu2', 'Tu3' ]
    color_list = ['orange', 'orange', 'darkgreen', 'orange', 'orange', 'orange', 'orange','orange', 'orange']

    T_len = len(trajectory_list)
    lab_len = len(label_list)
    col_len = len(color_list)
    #print("label list length ", lab_len)
    #print("color list length ", col_len)
    #print("trajectory list length ", T_len)
    
    if T_len > lab_len:
        diff = T_len - lab_len
        #print(diff)
        while diff > 0:
            label_list.append("T_extra")
            diff = T_len - len(label_list)
    
    if T_len > col_len:
        last_el = color_list[-1]
        diff = T_len - lab_len
        #print(diff)
        while diff >0:
            color_list.append(last_el)
            diff = T_len - len(color_list)            
   
    target_set_x2 = np.linspace(target_interval[0][1], target_interval[1][1], num=50)
    target_set_x1 = np.linspace(target_interval[0][0], target_interval[0][0], num=50)
    #print("target set", target_set_x2)
    #print(target_set_x1)
    target_set = [target_set_x1, target_set_x2]
    #print(len(trajectory_list), len(label_list), len(color_list))
    paper_plot_constraint_reach_avoid_target(target_set, trajectory_list,\
                                                     label_list,color_list,\
                                                     save,1,folder,\
                                                     filename=heading, title=heading)
    

def plot_region_boundaries_with_control(target_interval ,trajectory_list, current_time, folder, con_traj = "N/A", save=False, heading="Region of Stablisability" ):

    label_list = ['Td', 'Ta', 'T_constraint', 'upper_extreme','lower_exterme', 'Tlower', 'Tu1', 'Tu2', 'Tu3' ]
    color_list = ['orange', 'orange', 'darkgreen', 'orange', 'orange', 'orange', 'orange','orange', 'orange']

    T_len = len(trajectory_list)
    lab_len = len(label_list)
    col_len = len(color_list)
    #print("label list length ", lab_len)
    #print("color list length ", col_len)
    #print("trajectory list length ", T_len)
    
    if T_len > lab_len:
        diff = T_len - lab_len
        #print(diff)
        while diff > 0:
            label_list.append("T_extra")
            diff = T_len - len(label_list)
    
    if T_len > col_len:
        last_el = color_list[-1]
        diff = T_len - lab_len
        #print(diff)
        while diff >0:
            color_list.append(last_el)
            diff = T_len - len(color_list)            
   
    target_set_x2 = np.linspace(target_interval[0][1], target_interval[1][1], num=50)
    target_set_x1 = np.linspace(target_interval[0][0], target_interval[0][0], num=50)
    #print("target set", target_set_x2)
    #print(target_set_x1)
    target_set = [target_set_x1, target_set_x2]
    #print(len(trajectory_list), len(label_list), len(color_list))
    
    paper_plot_constraint_reach_avoid_target(target_set, trajectory_list,\
                                                     label_list,color_list,\
                                                     save,1,folder,\
                                                     filename=heading,\
                                                     title=heading,\
                                                     ct = con_traj    )
    



def paper_plot_constraint_reach_avoid_target(target_set, trajectory_list,label_list,\
                                                color_list,\
                                                save=True, marker_size=1,\
                                                filepath= "paper/",\
                                                filename= "sets",\
                                                title= "default title",
                                                ct="N/A"):
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
    
    lower_constraint_x1 = np.linspace(0, 1, num=50)
    lower_constraint_x2 = np.linspace(0, 0, num=50)

    
    if save == True:
        fig, ax = plt.subplots(dpi=600)
        #fig = plt.figure(dpi=600)
    else:
        fig, ax = plt.subplots()
    
    #if statement to 
    #print(trajectory_list[0])       
    
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
    
        
    plt.plot(lower_constraint_x1, lower_constraint_x2, ms=5, color='darkgreen')   
    
    for thing in things_to_plot:
        x = np.array(thing[0], dtype=np.float32)
        y = np.array(thing[1], dtype=np.float32)
        #print(x)
        plt.plot(x, y, ms=marker_size, color=thing[3], label=thing[2])
        #plt.plot(0.9, 3, ms=5, color='red', label=thing[2])  
        i =  i + 1
        
        
    set_start_x1 = np.linspace(0, 0, num=50)
    set_start_x2 = np.linspace(0, 3, num=50)
    
    plt.plot(set_start_x1, set_start_x2, ms=20, color='orange')    
    
    #print(x2)
    plt.plot(target_set[0], target_set[1], ms=5, color='red', label=thing[2])   
    if ct =="N/A":
        pass
    else:
        x1ct = [x[0] for x in ct]
        x2ct = [x[1] for x in ct]        
        plt.plot(x1ct, x2ct, ms=5, color='purple')        
    
    plt.plot([0.1],[2.0], 'or',ms=5,color='blue')
             
    custom_lines = [Line2D([0], [0], color='darkgreen', lw=4),\
                    Line2D([0], [0], color='orange', lw=4),\
                    Line2D([0], [0], color='red', lw=4),\
                    Line2D([0], [0], color='purple', lw=4),\
                    Line2D([0], [0], marker='o', color='w', label='Scatter',\
                          markerfacecolor='blue', markersize=10)]
        
    ax.legend(custom_lines, ['$\mathcal{X}$', '$\mathcal{R}(\mathcal{X}_{T})$', '$\mathcal{X}_{T}$', '$\mathcal{T}_f (x_0, \lambda(x))$', '$x_0$'],  prop={'size': 10})

    #plt.grid()
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
    







        
        
        
        
        
def plot_controller(trajectory_list, current_time, boundary_list_len, save=False):
    
    plotter = two_dof_robot_data_visualisation(current_time)

    label_list = ['T', 'T', 'T', 'T','T', 'Tu1', 'Tu2', 'Tu3', 'Tu4' ]
    #color_list = ['orange', 'orange', 'orange', 'orange', 'orange', \
                  #'orange', 'orange','orange','orange','orange', \
                      #'orange','orange','orange','orange','orange','orange',\
                          #'purple', 'purple']

    boundary_color = 'orange'
    control_color = 'purple'
    color_list = []
    i=0

    while i < boundary_list_len:
        if i == 0:
            color_list = [boundary_color]
        else:
            color_list. append(boundary_color)
        i = i + 1
        
    color_list.append(control_color)
    
    T_len = len(trajectory_list)
    lab_len = len(label_list)
    col_len = len(color_list)
    #print("label list length ", lab_len)
    #print("color list length ", col_len)
    #print("trajectory list length ", T_len)
    
    if T_len > lab_len:
        diff = T_len - lab_len
        #print(diff)
        while diff > 0:
            label_list.append("T_extra")
            diff = T_len - len(label_list)

    if T_len > col_len:
        last_el = color_list[-1]
        diff = T_len - lab_len
        #print(diff)
        while diff >0:
            color_list.append(last_el)
            diff = T_len - len(color_list)            
    
    print(len(color_list))
    print(len(label_list))
    plotter.overlay_trajectories_with_admissible_region("N/A",\
                                                        trajectory_list,\
                                                        label_list,color_list,\
                                                        save,1,filepath="paper_plots/",\
                                                        filename="controller_plot_8_1", title="controller")
           
def plot_control_type_1(control_trajectory, \
                        bounds, \
                        target_interval,\
                        filepath="paper_plots/",\
                        save=False):    
       

    lower_guide = bounds[0]
    upper_guide = bounds[1]
    
    plot_guides_with_control(control_trajectory,\
                                    upper_guide,\
                                    lower_guide,\
                                    save,\
                                    target_interval,\
                                    marker_size = 1,\
                                    filepath="paper_plots/",\
                                    filename="controller_trajectory_type_1")    





def plot_guides_with_control(control_trajectory,\
                                        upper_guide,\
                                        lower_guide,\
                                        save,\
                                        target_interval,\
                                        marker_size = 10,\
                                        filepath="paper_plots/",\
                                        filename="controller_plot"):
        
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

    target_set_x2 = np.linspace(target_interval[0][1], target_interval[1][1], num=50)
    target_set_x1 = np.linspace(target_interval[0][0], target_interval[1][0], num=50)
    
    set_start_x1 = np.linspace(0, 0, num=50)
    set_start_x2 = np.linspace(0, 3, num=50)
    
    if save == True:
        fig, ax = plt.subplots(dpi=600)
    else:
        fig, ax = plt.subplots()

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
        plt.plot(x1, x2, ms=3*marker_size, color='orange')                            
    
    if lower_guide == "N/A":
        pass
    else:
        x1 = [x[0] for x in lower_guide]
        x2 = [x[1] for x in lower_guide]
        plt.plot(x1, x2, ms=3*marker_size, color='orange')                
    
    plt.plot(target_set_x1, target_set_x2, ms=20, color='red')
    plt.plot(set_start_x1, set_start_x2, ms=20, color='orange')    
    
    custom_lines = [Line2D([0], [0], color='orange', lw=4),\
                    Line2D([0], [0], color='red', lw=4),\
                    Line2D([0], [0], color='purple', lw=4)]
        
    ax.legend(custom_lines, ['$Bd(\mathcal{R}(x^{*}))$', '$x^*$', '$\mathcal{T}$'],  prop={'size': 12}, loc='upper left')

    plt.grid()
    plt.xlabel("$x_1$")
    plt.ylabel("$x_2$")
    plt.xticks(np.arange(0, 1.1, step=0.1))
    plt.yticks(np.arange(0, 10, step=1)) 
    plt.title("Trajectory within the Reach Avoid set")

    if save == True:
        fig.savefig("plots/" + filepath + filename + ".png", format="png")
        fig.savefig("plots/" + filepath + filename + ".svg", format="svg")
        plt.close()
    else:
        plt.show()        


def plot_control_type_2(control_trajectory,\
                                        control_bounds,\
                                        admissible_bounds,\
                                        target_interval,\
                                        filepath="paper_plots/",\
                                        save=False):
    
    
    lower_guide = control_bounds[0]
    upper_guide = control_bounds[1]
    
    lower_admissible = admissible_bounds[0]
    upper_admissible = admissible_bounds[1]  

    paper_plot_control_type_2(control_trajectory,\
                                    upper_guide,\
                                    lower_guide,\
                                    upper_admissible,\
                                    lower_admissible,\
                                    save,\
                                    target_interval,\
                                    marker_size = 1,\
                                    filepath="paper_plots/",\
                                    filename="controller_trajectory_type_2",\
                                    title="Control trajectory using control guides wwithin the reach avoid set")        
    

def paper_plot_control_type_2(control_trajectory,\
                        upper_guide,\
                        lower_guide,\
                        upper_admissible,\
                        lower_admissible,\
                        save,\
                        target_interval,\
                        marker_size = 1,\
                        filepath="paper_plots/",\
                        filename="controller_plot",\
                        title="controller"):


    if save == True:
        fig, ax = plt.subplots(dpi=600)
    else:
        fig, ax = plt.subplots()


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
    
    if control_trajectory == "N/A":
        pass
    else:
        x1 = [x[0] for x in control_trajectory]
        x2 = [x[1] for x in control_trajectory]
        plt.plot(x1, x2, ms=3*marker_size, color='purple')                
        

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
    
    
    
    set_start_x1 = np.linspace(0, 0, num=50)
    set_start_x2 = np.linspace(0, 3, num=50)
    
    plt.plot(set_start_x1, set_start_x2, ms=20, color='orange')   
    
    
    target_set_x2 = np.linspace(target_interval[0][1], target_interval[1][1], num=50)
    target_set_x1 = np.linspace(target_interval[0][0], target_interval[1][0], num=50)
    
    plt.plot(target_set_x1, target_set_x2, ms=5, color='red')   
        
    
    custom_lines = [Line2D([0], [0], color='orange', lw=4),\
                    Line2D([0], [0], color='purple', lw=4),\
                    Line2D([0], [0], color='blue', lw=4)]
        
    ax.legend(custom_lines, ['$Bd(\mathcal{R}(x^{*}))$', '$\mathcal{T}$', 'control guides'],  prop={'size': 10})


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




def plot_control_type_3(control_trajectory,\
                                        control_bounds,\
                                        admissible_bounds,\
                                        target_interval,\
                                        filepath="paper_plots/",\
                                        save=False):
    
    
    lower_guide = control_bounds[0]
    upper_guide = control_bounds[1]
    
    lower_admissible = admissible_bounds[0]
    upper_admissible = admissible_bounds[1]  

    paper_plot_control_type_3(control_trajectory,\
                                    upper_guide,\
                                    lower_guide,\
                                    upper_admissible,\
                                    lower_admissible,\
                                    save,\
                                    target_interval,\
                                    marker_size = 1,\
                                    filepath="paper_plots/",\
                                    filename="controller_trajectory_type_3",\
                                    title="Control trajectory using a single control guide within the reach avoid set")        
    

def paper_plot_control_type_3(control_trajectory,\
                        upper_guide,\
                        lower_guide,\
                        upper_admissible,\
                        lower_admissible,\
                        save,\
                        target_interval,\
                        marker_size = 1,\
                        filepath="paper_plots/",\
                        filename="controller_plot",\
                        title="controller"):


    if save == True:
        fig, ax = plt.subplots(dpi=600)
    else:
        fig, ax = plt.subplots()


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
    
    if control_trajectory == "N/A":
        pass
    else:
        x1 = [x[0] for x in control_trajectory]
        x2 = [x[1] for x in control_trajectory]
        plt.plot(x1, x2, ms=3*marker_size, color='purple')                
        

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
    
    
    
    set_start_x1 = np.linspace(0, 0, num=50)
    set_start_x2 = np.linspace(0, 3, num=50)
    
    plt.plot(set_start_x1, set_start_x2, ms=20, color='orange')   
    
    
    target_set_x2 = np.linspace(target_interval[0][1], target_interval[1][1], num=50)
    target_set_x1 = np.linspace(target_interval[0][0], target_interval[1][0], num=50)
    
    plt.plot(target_set_x1, target_set_x2, ms=5, color='red')   
        
    
    custom_lines = [Line2D([0], [0], color='orange', lw=4),\
                    Line2D([0], [0], color='purple', lw=4),\
                    Line2D([0], [0], color='blue', lw=4)]
        
    ax.legend(custom_lines, ['$Bd(\mathcal{R}(x^{*}))$', '$\mathcal{T}$', 'control guide'],  prop={'size': 10}, loc='upper left')


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



def plot_control_type_4(control_trajectory,\
                                        control_bounds,\
                                        admissible_bounds,\
                                        target_interval,\
                                        filepath="paper_plots/",\
                                        save=False):
    
    
    lower_guide = control_bounds[0]
    upper_guide = control_bounds[1]
    
    lower_admissible = admissible_bounds[0]
    upper_admissible = admissible_bounds[1]  

    paper_plot_control_type_3(control_trajectory,\
                                    upper_guide,\
                                    lower_guide,\
                                    upper_admissible,\
                                    lower_admissible,\
                                    save,\
                                    target_interval,\
                                    marker_size = 1,\
                                    filepath="paper_plots/",\
                                    filename="controller_trajectory_type_4",\
                                    title="Control trajectory using a single control guide within the reach avoid set")        
    

def paper_plot_control_type_4(control_trajectory,\
                        upper_guide,\
                        lower_guide,\
                        upper_admissible,\
                        lower_admissible,\
                        save,\
                        target_interval,\
                        marker_size = 1,\
                        filepath="paper_plots/",\
                        filename="controller_plot",\
                        title="controller"):


    if save == True:
        fig, ax = plt.subplots(dpi=600)
    else:
        fig, ax = plt.subplots()


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
    
    if control_trajectory == "N/A":
        pass
    else:
        x1 = [x[0] for x in control_trajectory]
        x2 = [x[1] for x in control_trajectory]
        plt.plot(x1, x2, ms=3*marker_size, color='purple')                
        

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
    
    
    
    set_start_x1 = np.linspace(0, 0, num=50)
    set_start_x2 = np.linspace(0, 3, num=50)
    
    plt.plot(set_start_x1, set_start_x2, ms=20, color='orange')   
    
    
    target_set_x2 = np.linspace(target_interval[0][1], target_interval[1][1], num=50)
    target_set_x1 = np.linspace(target_interval[0][0], target_interval[1][0], num=50)
    
    plt.plot(target_set_x1, target_set_x2, ms=5, color='red')   
        
    
    custom_lines = [Line2D([0], [0], color='orange', lw=4),\
                    Line2D([0], [0], color='purple', lw=4),\
                    Line2D([0], [0], color='blue', lw=4)]
        
    ax.legend(custom_lines, ['$Bd(\mathcal{R}(x^{*}))$', '$\mathcal{T}$', 'control guide'],  prop={'size': 10}, loc='upper left')


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


def plot_L_v_x1(list_to_plot, save=False,filepath="paper_plots/",\
                    filename="actuation_level_8_1", title="list of things"):
    
    if save == True:
        fig, ax = plt.subplots(dpi=600)
    else:
        fig, ax = plt.subplots()
    x1 = [x[1][0] for x in list_to_plot]
    L = [x[0] for x in list_to_plot]
    
    #x = np.arange(1, len(list_to_plot)+1, 1)
    #y = list_to_plot

    plt.plot(x1, L, 'or',ms=1,color='red')
    plt.grid()
    plt.xlabel("$x_1$")
    plt.ylabel("$\lambda$") 
    plt.title(title)
    #ax.set(ylim=(0, 1))
    if save == True:
        fig.savefig("plots/" + filepath + filename + ".svg", format="svg")
        fig.savefig("plots/" + filepath + filename + ".png", format="png")
        plt.close()
    else:
        plt.show()    
        
        
#def plot_joints():
def plot_joints(q0, q1, q2, q3, q4, q5,\
                N, save=False,filepath="paper_plots/",\
                    filename="joint_plot", title="list of things"):
    
    if save == True:
        fig, ax = plt.subplots(dpi=600)
    else:
        fig, ax = plt.subplots()
    #x1 = [x[1][0] for x in list_to_plot]
    #L = [x[0] for x in list_to_plot]
    
    #x = np.arange(1, len(list_to_plot)+1, 1)
    #y = list_to_plot
    #print(len(q0), len(N))
    plt.plot(N, q0, 'or',ms=1,color='red')
    plt.plot(N, q1, 'or',ms=1,color='red')
    plt.plot(N, q2, 'or',ms=1,color='red')
    plt.plot(N, q3, 'or',ms=1,color='red')
    plt.plot(N, q4, 'or',ms=1,color='red')    
    plt.plot(N, q5, 'or',ms=1,color='red')
    plt.grid()
    plt.xlabel("$x_1$")
    plt.ylabel("$\lambda$") 
    plt.title(title)
    #ax.set(ylim=(0, 1))
    if save == True:
        fig.savefig("plots/" + filepath + filename + ".svg", format="svg")
        fig.savefig("plots/" + filepath + filename + ".png", format="png")
        plt.close()
    else:
        plt.show()