# -*- coding: utf-8 -*-
"""
Created on Sun Mar 22 11:49:26 2020
This function will take a manipulator model and work out control actions

@author: Ryan
"""
from math import sqrt
import numpy as np
import matplotlib.pyplot as plt
from my_math import fit_curve, intersection
from my_visualising import add_to_plot

class path_dynamics_controller():
    """
    This class will contain the methods for designing a controller for moving
    the robot along the state space. I need to 
    """
    
    def __init__(self, bounds, s, sd):
        #print("initialised")
        #pass        
        self.bounds = bounds
        self.s = s 
        self.sd = sd



        #print(bounds)
        #print("yeeeeeee")

  
    def simple_time_optimal_controller(self, initial, final):
        """
        This is a method that will output a valid trajectoy of (s, sd) points 
        from which a joint torque input sequence can be derived
        It works by using a knowledge of actuator limits in the state space
        Given an intial and final condition the algorithm will try and find the fastest trajectory 
        
        Arguments:
            initial- (s_start, sd_start) initial position
            final - (s_end, sd_end) - final position
            bounds -  [(B01,B02, Ms0),(B21,B22, Ms1),(B31,B32, Ms2),...(Bn1, Bn2, Msn)]  expressions for the actuator limits
            B01 is bound 1 on actuator zero
            B02 is bound 2 on actuator zero
            Ms0 is the value that decides the upper and lower at some s, sd number
        return:
            trajectory - [(s1,sd1), ... , (sn, sdn) ]
            trajectory from s_start->s_end
        
        
        
        The process is outined as:
            
            1- Start at final position and integrate backwards sdd == -L until U-L <= 0 (velocity limit curve hit) 
              or s==0 (max acceleration decelleration possible)
            
            2- Start at intial and integrate forward sdd == U until either:
                U-L <= 0 (velocity limit or inadmissable region found) or s >= 1
            
            3- If the inadmissable region is hit in step 2:
                    take steps backwards taking the sdd == L and run forwards until inadmissable region is hit or sd == 0
                    repeat until it isnt hit and at the point where the boundy of this region 
                    find the point on this curve that is closest to the in admissable region (mark it as a switching point)
            
            4- switch sdd == U at point in step 3, integrat forward until either:
                if U-L <=0: repeat step 3:
                
                else if curve formed by step 1 is intersected mark this as a switching point for decelleration
            
            5- output a valid serise of points along the trajectories calculated 
                (these can readily be used to calculater the neccessary  actuator torques)
        """
        
        step_size = 0.1
        #1 backward integrate sdd = -L
        #print(self.bounds)
        #perform the backward integration first from final position
        seg_final = self.integrate_motion_time_optimal(final, step_size, 'backwards' )
        seg_final.reverse()
        

        #2 forward integrate sdd == +U
        
        #seg_initial = self.integrate_motion_time_optimal(initial, step_size, 'forwards')
        

        #intersection_point = self.find_intersections(seg_initial, seg_final, step_size)
        
         
        #all_segs = [seg_initial, seg_final]
        
        #print(all_segs)
        
        trajectory = seg_final#self.connect_trajectories(all_segs, intersection_point)
        intersection_point = [(0,0)]
        return trajectory, intersection_point     
        
        
        
        #x, p2 = fit_curve(seg2)
        #plt.plot(x,p2)
        """
        #print(seg1)
        x_val = [x[0] for x in seg2]
        y_val = [x[1] for x in seg2]
         
        x_val = np.array(x_val, dtype=float)
        #n = np.linspace(-20,20,10)
        #print(type(x_val), type(n))
        y_val = np.array(y_val ,dtype=float)
        n = np.linspace(0, 1, 100)
        z = np.polyfit(x_val,y_val,1)
        p2 = np.poly1d(z)
        plt.plot(n, p2(n))
        
        r = np.roots(p1-p2)
        e = np.polyval(p2, r)
        #print(r, e)
        intersection = (r[0], e[0])
        print(intersection)
        plt.plot(intersection[0], intersection[1], 'or',ms=10)
        
        #plt.show()
        """
        #plot
        
        #3 backstep until tangent found sdd == -L


        #4 goto step 2
        
        
        #5 if curve formed in step one is intersected end else goto 3
        
        #6 repeat 5 until end curve is found
        
        
        
        #trajectory = 1
        #print("we got the controller designed")
        #trajectory = seg1 +seg2
        #return trajectory, intersection_point
        
    def find_intersections(self, seg1, seg2, step_size):
        """
        this method takes in two lists of points and decides if they intersect
        Arguments:
            seg1 & seg2 - list of tuples [(x1, y1), ... , (xn, yn)]
        return:
                        
        """
        c = 5 #varioble to tue intersection speration
        
        i = 0
        j = 0
        
        seg1_section = []
        seg2_section = []
        
        
        while(i < len(seg1)):
        
            while(j < len(seg2)):
                
                seperation = sqrt((seg1[i][0] - seg2[j][0])**2 + (seg1[i][1] - seg2[j][1])**2)
                
                
                if seperation < c*step_size:
                                    
                    if len(seg1_section) == 0:
                        seperation_list = [seperation]
                        seg1_section = [seg1[i]]
                        seg2_section = [seg2[j]]
                        
                    else:
                        seperation_list.append(seperation)
                        seg1_section.append(seg1[i])
                        seg2_section.append(seg2[j])

                    #print(seperation, seg1[i], seg2[j])
                
                j = j + 1
            
            
            j = 0
            i = i + 1


        #val, idx = min((val, idx) for (idx, val) in enumerate(seperation_list))
        try:
            index_min = np.argmin(seperation_list)#get the closest points
            return [seg1_section[index_min]]
        except:
            return [(-1,-1)]#no intersections found
        
        #raise Exception("lines do not intersect")
            
        #print(val, idx)
        #print(seg1_section)
        #print("ewfbiuewrbgerbiuogrvb")
        #print(seg2_section)
        
        #x, p1 = fit_curve(seg1_section)
        #plt.plot(x,p1)
        
        #x, p2 = fit_curve(seg2_section)
        #plt.plot(x,p1)
        
        
        #point = intersection(p1,p2)
        
        #plt.plot(point, 'or',ms=10)
        
        #plt.show()


    def connect_trajectories(self, segs, intersection):
        """
        This method takes in a few different segments and an intersection point
        
        return
            trajectory - [(s1,sd1), ... , (sn, sdn) ]
        
        """
        #print(intersection)
        
        i = 0
        j = 0
        trajectory = []
        not_finished = True
        
        while(not_finished):
            
            if len(trajectory) == 0:
                trajectory = [segs[j][i]]          
            else:
                trajectory.append(segs[j][i])   
            
            
            i = i + 1
                        
            #if we pass the intersection point
            if j == 0:
                if segs[j][i][0] >  intersection[0][0]:
                    j = j + 1 
                    i = 0
                    #increment i until interse4ction point
                    while segs[j][i][0] <  intersection[0][0]:
                        i = i + 1
                        
            
            #print(i, j, segs[j][i][0])
            if segs[j][i][0] == 1:
                not_finished = False
                    
                    
                    
        
        return trajectory


    def integrate_motion_time_optimal(self, pos, step_size, direction="backwards"):
        """
        Arguments:
            start_coordinate - (x, y)
            bounds -  [(B01,B02, Ms0),(B21,B22, Ms1),(B31,B32, Ms2),...(Bn1, Bn2, Msn)]  expressions for the actuator limits
                        B01 is bound 1 on actuator zero
                        B02 is bound 2 on actuator zero
                        Ms0 is the value that decides the upper and lower at some s, sd number
        return -
            trajectory - [(s1,sd1), ... , (sn, sdn) ]
        """
               
        trajectory = [pos]
        i = 0
        
        s = pos[0]
        sd = pos[1]
    
        integration_complete = False

        while(integration_complete == False and i<1000):
            
            #calculate -L
            #print(current_pos)
            L, U = self.calc_upper_lower_bound_values(pos)
            s = pos[0]
            sd = pos[1]
            
            if direction == "backwards":
                delta_s, delta_sd = self.calc_change_s_sd(-sd, -L, step_size)#integrate backwards
            elif direction == "forwards":
                delta_s, delta_sd = self.calc_change_s_sd(sd, U, step_size)#integrate forwards


            s = s + delta_s
            sd = sd + delta_sd
            pos = (s, sd)
            #print(pos)
            trajectory.append(pos)

            
            if L > U or s < 0 or s > 1 or sd < 0:
                integration_complete = True
                
            #print(trajectory)
            i=i+1
        #print('i=',i, )
        


        return trajectory

    def calc_change_s_sd(self, s_vel, sd_vel, step_size):
        """
        method will take in s and sd changes and ensure that the magnitude of the step
        is equal so some stepsize
        return- delta_s (value to be added to current s)
                delta_sd (value to be added to current sd)
        
        """
        #account for all cases
        if abs(s_vel) > 0 and abs(sd_vel) > 0:
            
            length = sqrt(s_vel**2 + sd_vel**2)
            b = step_size/length
            
            delta_s = (s_vel*b)#step_size)/length
            delta_sd = (sd_vel*b)#step_size)/length
            
            #c = sqrt(delta_s**2 + delta_sd**2)
            
            #print(c, length, delta_s, delta_sd)
        elif  abs(s_vel) > 0 and abs(sd_vel) == 0:
            
            delta_s = step_size
            delta_sd = 0
        
        elif abs(sd_vel) > 0 and abs(s_vel) == 0:
            delta_sd = step_size
            delta_s = 0
        else:
            raise Exception("something is wrong with calc_change_sd (s_vel, sd_vel)=", (s_vel, sd_vel))


        return delta_s, delta_sd

    
    def dsd_ds(self, sd, sdd):
        """
        Function to calculate how d(sdot)/ds, this will essentially decide
        on the next s s dot coorcinate of any control algorithm applied 
        
        Arguments:
            sd -  ds/dt
            sdd - d^2s/dt^2
        return
            dsd/ds 
        """
        
        
        dsd_ds = abs(sqrt(sd**2 + sdd**2))
        
        #choose -ve square root
        if sdd < 0:
            dsd_ds = -1*(dsd_ds)
            
        return dsd_ds
  
        

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

    def evaluate_Ek_s(self, Ek_s, s, sd):
        """
        Method to take in the kinetic energy function of teh robot and oyhre
        
            Arguments:
                Ek_s  and expression of teh form f(s, sd)
        
            return
                Ek_val = Ek_evaluated at (s,sd)
        """
        Ek_val = Ek_s.subs([(self.s, s), (self.sd, sd)])
        
        
        
        return Ek_val
























