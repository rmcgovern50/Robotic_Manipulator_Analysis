# -*- coding: utf-8 -*-
"""
This is a file that will contain models for specific robots I wish to study
Each robot, robot specific parts will be coded here
These are all child classes the the path_dynamics_analysis class which helps to 
visualise the state space in a general way
"""

from sympy import symbols, Matrix, sin, cos, diff, Abs, Subs
from my_sorting import combine_to_tuples
from path_dynamics_analysis import path_dynamics
from path_dynamics_control import path_dynamics_controller as pdc
import math as m
from matplotlib import pyplot
from my_visualising import simple_polar_plot, simple_plot, add_to_plot

class revolute_prismatic(path_dynamics, pdc):
    pass
    """
    This class is for the case specific parts of revolute slide robot
    The path_dynamics analysis class will use the information here to
    calculate admissable regions of the state space along a path
    """
    
    def __init__(self, end_effector_mass, joint_limits):
        """
        Initialise the robot with it's parameters
        Arguments: 
        end_effector_mass (mass of end effector in kg)
        joint_limits list of tuples describing max joint limits of form
        [(u1min,u1max), (u2min,u2max), ... , (unmin,unmax)]
        the joint limits will always be a list of 2 tuples in this case
        """
        self.type = "revolute prismatic robot (polar robot)"
        
        self.s, self.sd, self.sdd = symbols('s sd sdd')
        self.joint_limits = joint_limits

        self.m, self.u1min, self.u1max, self.u2min, self.u2max,\
        = symbols('m u1min u1max, u2min u2max')#make symbols for all the varables for symbolic calculations
        
        self.q1, self.q2, self.q1d, self.q2d, self.q1dd, self.q2dd, self.u1, self.u2 \
        = symbols('q1 q2 q1d q2d q1dd q2dd u1, u2')
        
        #list that is of the form that allows easy substitution of robot parameters.
        symbols_to_sub = [self.m, self.u1min, self.u1max, self.u2min, self.u2max]
        values_to_sub = [end_effector_mass, joint_limits[0][0], joint_limits[0][1], joint_limits[1][0], joint_limits[1][1]]
        constants_to_sub = combine_to_tuples(symbols_to_sub, values_to_sub)
        self.dynamics()
        
        super(revolute_prismatic, self).__init__(constants_to_sub)
        
    def dynamics(self):
        
        """
        Form the dynamical equations in matrix form, return the result
        """
        #define moments of inertia              
        #define inertia matrix
        
        #there is a serious mistake as q1 and q2 are defined the wrong way around
        g = 9.81 #ms^-2
        
        M11 = self.m*(self.q2**2)  
        M12 = 0        
        M21 = 0        
        M22 = self.m
        
        self.M = Matrix([[M11, M12], [M21, M22]])#form inertia matrix
        
        C11 = 0#2*self.m*self.q2*self.q2d
        C12 = 0
        C21 = -1*self.m*self.q2*self.q1d       
        C22 = 0
        
        self.C = Matrix([[C11, C12],[C21, C22]])
        
        G1 = self.m*g*self.q2*cos(self.q1)
        G2 = self.m*g*sin(self.q1)
        
        self.g = Matrix([[G1], [G2]])
        
        self.q = Matrix([[self.q1],[self.q2]])
        
        self.qd = Matrix([[self.q1d], [self.q2d]])

        self.qdd = Matrix([[self.q1dd], [self.q2dd]])
        
        self.inputs = Matrix([[self.u1], [self.u2]])
        
        return self.M, self.C, self.g
    
    
    def forward_kinematics(self, joints):
        """
        Arguments:
            function takes in list of joint positions of the form:
            joints = [q1. q2]
            q1 in radians, q2 in length units
        
        returns:
            end effector pose
            pose = [x, y]
            x in length units, y in length units
        """
        q1 = joints[0]
        q2 = joints[1]
        
        x = q2*m.cos(q1)
        y = q2*m.sin(q1)
        
        pose = [x, y]
        
        return pose
    def inverse_kinematics(self, pose, return_degrees=0):
        """
        Arguments:
            function takes in end effector pose
            pose = [x, y]
            return_degrees if set to 1 returns q1 as degrees instead of radians
            
            
        return:
            joint positions as
            joint_positions = [q1, q2]
            where q1 is in radians, q2 in length units
        """
        x = pose[0]
        y = pose[1]
        
        """
        ======quadrant layout======
                    |
        quadrant 2  |   quadrant 1 
            ---------------------
                    |
        quadrant 3  |   quadrant 4
    
        """
        if x!= 0 or y!= 0:
                
            q2 = m.sqrt(x**2 + y**2) # caculate q2
                
            #now apply diffent rules dependant on the quadrant as tab function has issues
            if x > 0 and y > 0:
                #quadrant 1
                q1 = m.atan(y/x)
            elif x < 0 and  y > 0:
                #quadrant 2
                q1 = m.pi + m.atan(y/x)
            elif x < 0 and  y < 0: 
                #quadrant 3
                q1 = m.pi + m.atan(y/x) 
            elif x > 0 and  y < 0:
                #quadrant 4
                q1 = m.atan(y/x)
            elif x > 0 and y == 0:
                #quadrant 1-4 boundry
                q1 = 0
            elif x < 0 and y == 0:
                #quadrant 2-3 boundry
                q1 = 0
            elif x == 0 and y > 0:
                #quadrant 1-2 boundry
                q1 = m.pi/2
            elif x == 0 and y < 0:
                #quadrant 3-4 boundry
                q1 = -m.pi/2
                
            if return_degrees == 1:
                #simply return the value in degrees
                q1 = m.degrees(q1)
        
        else:
            print("If pose is 0,0 any value of q1 is valid")
            print("chosen value is 0")
            q1 = 0
            q2 = 0

        joint_positions = [q1,q2]
        return joint_positions
    
    def straight_line_parameterisation(self, start, end):
        """
        Arguments:
            start = [x1, y1]
            end = [x2, y2]
            
        return:
            qs = [q1(s), q2(s)]
            
        Description:
        Function works by taking in two points in the cartesian space
        the equation of a straight line is:
            y = mx + c in cartesian space
        This is a plar manipulator so we can define the following transormation for any point in space
        y = q2sinq1
        x = q2cosq1
        
        so a line can be defined as:
            
        q2*cosq1 = q2*m*sinq1 + c
        or:
        q2 = c/(cosq1 - msinq1)
        
        WARNING, this equation will only work correctly first quadrant:
            -
        as 
        
        """

        #get m
        x1 = start[0]
        y1 = start[1]
        x2 = end[0]
        y2 = end[1]
        
        #if x1 <= 0 or y1 <= 0 or x2 <= 0 or y2 <= 0:
        #    raise Exception("need to work out code for doing any quadrants that aren't the first one (top left)")
            
        #get the start and end value of q1
        q1_start = self.inverse_kinematics(start)[0] #start angle
        q1_end = self.inverse_kinematics(end)[0] #end angle
        q1s = q1_start + self.s*(q1_end - q1_start) # linear interpolation between start and end angles
        
        if x1 -  x2 != 0:
            m = (y1 - y2)/(x1 -  x2)
            #print(m)
            c = y1 - m*x1
            
            q2s = c/(sin(q1s) - m*cos(q1s))# enforce rule to make q2 vary so that the end effector follows an sreaight line
            
        elif x1 == x2:
            #we have a line with infinte gradient
            q2s = x1/(cos(q1s))
            
        else:
            raise Exception("gradient cannot be calculated")
            
            
        qs = [q1s, q2s]
        return qs
        
    def plot_end_effector_trajectory(self, qs, increment=0.1,\
                                     plot_q1_against_s=0, plot_q2_against_s=0, plot_polar_q1s_against_q2s=0, plot_q1_against_q2=0):
        """
        Arguments:
            qs - list of parameterised trajectories
            increment - set increment size
            
        return:
            nothing, simply plot a polar map
            
        """
        s = 0
        coordinates = []
        s_axisq1 = []
        s_axisq2 = []
        q1_v_q2 = []
        
        while(s < 1):
            if s == 0:
                coordinates = [(qs[0].subs([(self.s, s)]), qs[1].subs([(self.s, s)]))]
                s_axisq1 = [(s, qs[0].subs([(self.s, s)]))]
                s_axisq2 = [(s, qs[1].subs([(self.s, s)]),s)]
            else:
                coordinates.append((qs[0].subs([(self.s, s)]), qs[1].subs([(self.s, s)])))
                s_axisq1.append((s, qs[0].subs([(self.s, s)])))
                s_axisq2.append((s, qs[1].subs([(self.s, s)])))
            s = s + increment
            
        #print(coordinates)
        
        if plot_q1_against_s == 1:
            simple_plot(s_axisq1, 's', 'q1', 1)
        if plot_q2_against_s == 1:
            simple_plot(s_axisq2, 's', 'q2', 1)        
        if plot_polar_q1s_against_q2s == 1:
            simple_polar_plot(coordinates)
        if plot_q1_against_q2 == 1:
            simple_plot(coordinates, 'q1', 'q2', 1)
    

    def Run_path_dynamics(self, path_straight_line, s_lims, sd_lims):
        """
        Description-
            This is a method that uses many of the existing methods in this class
            to simply produce an admissable region plot amoung other information
            that can then be used to design controllers in this space
        
        Arguments:
            path_straight_line - [(xstart, ystart), (xend, yend)] - point to point movement coordinates
            s_lims - [start s, end s, s increment] - sampling rate for admissable region
            sd_lims - [start sd, end sd, sd increment] - sampling rate for admissable region
        
        return:
            region - list of tuples decribing the admissable region
            boundry_points - list of points decribing the boundry
            boundry expressions - list of expressions that when evaluated can get us the upper and lower limits on sdd
            plt - return plot of the admissable region
        """
        
     
        #repeat code used externally except simple_plot

        qs = self.straight_line_parameterisation(path_straight_line[0], path_straight_line[1])
        
        self.plot_end_effector_trajectory(qs, 0.01, 1, 1, 1, 1)
        
        q, qd,qdd, dqds, d2qds2  = self.path_parameterisation(qs)
        #print(q, qd,qdd, dqds, d2qds2)

        #get all the necessary matrices in terms of the parameterised matrices
        Mqs, Cqs, gqs = self.calc_qs_matrices(q, qd, qdd)
        #print(Mqs, Cqs, gqs)
        
        #form M(s), C(s), and g(s) as M(s)*sdd + C(s)*sd**2 + g(s) = t(s)
        Ms, Cs, gs = self.calc_s_matrices(Mqs, Cqs, gqs, dqds, d2qds2)
        #print(Ms, Cs, gs)
        
        #calculate bounds based on the path dynamics
        self.bounds = self.calc_bounds(Ms, Cs, gs)
        #print(bounds)

        #get big list of tangent cones corresponding to ead point in the state space 


        #tangent_cone = f(s, sd)
    def generate_time_optimal_trajectory(self):
        """
        method to take steps neccesary to design a time opeitmal control trajectory
        
        return:
            trajectory - [(s1, sd1),...,(sn,sdn) ]
        """
    
        #define grid
        s_lim = [0, 1, 0.1]
        sd_lim = [0,10, 0.1]
        
        #calculate admissable region
        admissable_region, boundry_points = self.calc_admissable(self.bounds, s_lim, sd_lim)

        plot = self.generate_state_space_plot(admissable_region, 1)
        #plot.show()
   
    
        add_to_plot(plot, [(0.2,6)])
        plot.show()
        self.simple_time_optimal_controller()
        
        #tangent_cone_components = control.generate_tangent_cone_components(self.bounds, s_lim, sd_lim)
        #print(tangent_cone_components)
        
        
        print("design controller here")
        return 1
            
      
        
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
#==============================================================================
            

class cartesian_robot(path_dynamics):
    """
    This class is for the case specific parts of cartesian robot
    The path_dynamics analysis class will use the information here to
    calculate admissable regions of the state space along a path
    """
    
    def __init__(self, link_masses, joint_limits):
        """
        Initialise the robot with it's parameters
        Arguments: 
        link_masses  of form [m1, m2]
        joint_limits list of tuples describing max joint limits of form
        [(u1min,u1max), (u2min,u2max), ... , (unmin,unmax)]
        The joint limits will always be a list of 2 tuples in this case
        """
        self.type = "revolute prismatic robot (polar robot)"
        
        self.s, self.sd, self.sdd = symbols('s sd sdd')
        self.joint_limits = joint_limits

        self.m1, self.m2, self.u1min, self.u1max, self.u2min, self.u2max,\
        = symbols('m1 m2 u1min u1max, u2min u2max')#make symbols for all the varables for symbolic calculations
        
        self.q1, self.q2, self.q1d, self.q2d, self.q1dd, self.q2dd, self.u1, self.u2 \
        = symbols('q1 q2 q1d q2d q1dd q2dd u1, u2')
        
        #list that is of the form that allows easy substitution of robot parameters.
        symbols_to_sub = [self.m1, self.m2, self.u1min, self.u1max, self.u2min, self.u2max]
        values_to_sub = [link_masses[0], link_masses[1], joint_limits[0][0], joint_limits[0][1], joint_limits[1][0], joint_limits[1][1]]
        constants_to_sub = combine_to_tuples(symbols_to_sub, values_to_sub)
        
        super().__init__(constants_to_sub)
        
    def dynamics(self):
        
        """
        Form the dynamical equations in matrix form, return the result
        """
        #define moments of inertia              
        #define inertia matrix
        
        #there is a serious mistake as q1 and q2 are defined the wrong way around
        g = 9.81 #ms^-2
        
        M11 = self.m1 + self.m2  
        M12 = 0        
        M21 = 0        
        M22 = self.m2
        
        self.M = Matrix([[M11, M12], [M21, M22]])#form inertia matrix
        
        C11 = 0
        C12 = 0
        C21 = 0      
        C22 = 0
        
        self.C = Matrix([[C11, C12],[C21, C22]])
        
        G1 = g*(self.m1 + self.m2)
        G2 = 0
        
        self.g = Matrix([[G1], [G2]])
        
        self.q = Matrix([[self.q1],[self.q2]])
        
        self.qd = Matrix([[self.q1d], [self.q2d]])

        self.qdd = Matrix([[self.q1dd], [self.q2dd]])
        
        self.inputs = Matrix([[self.u1], [self.u2]])
        
        return self.M, self.C, self.g
    
    
    def forward_kinematics(self, joints):
        """
        Arguments:
            function takes in list of joint positions of the form:
            joints = [q1. q2]
            q1 in length units, q2 in length units
        
        returns:
            end effector pose
            pose = [x, y]
            x in length units, y in length units
        """
        q1 = joints[0]
        q2 = joints[1]
        
        x = q1
        y = q2
        
        pose = [x, y]
        
        return pose
    def inverse_kinematics(self, pose):
        """
        Arguments:
            function takes in end effector pose
            pose = [x, y]
            return_degrees if set to 1 returns q1 as degrees instead of radians
            
            
        return:
            joint positions as
            joint_positions = [q1, q2]
            where q1 is in radians, q2 in length units
        """
        q1 = pose[0]
        q2 = pose[1]
        
        joint_positions = [q1,q2]
        return joint_positions
    
    def straight_line_parameterisation(self, start, end):
        """
        Arguments:
            start = [x1, y1]
            end = [x2, y2]
            
        return:
            qs = [q1(s), q2(s)]
            
        Description:
        Function works by taking in two points in the cartesian space
        the equation of a straight line is:
            y = mx + c in cartesian space
        This is a plar manipulator so we can define the following transormation for any point in space
        y = q2sinq1
        x = q2cosq1
        
        so a line can be defined as:
            
        q2*cosq1 = q2*m*sinq1 + c
        or:
        q2 = c/(cosq1 - msinq1)
        
        WARNING, this equation will only work correctly first quadrant:
            -
        as 
        
        """

        #get m
        x1 = start[0]
        y1 = start[1]
        x2 = end[0]
        y2 = end[1]
        
        #if x1 <= 0 or y1 <= 0 or x2 <= 0 or y2 <= 0:
        #    raise Exception("need to work out code for doing any quadrants that aren't the first one (top left)")
            
        #get the start and end value of q1
        q1_start = self.inverse_kinematics(start)[0] #start angle
        q1_end = self.inverse_kinematics(end)[0] #end angle
        
        q2_start = self.inverse_kinematics(start)[1] #start angle
        q2_end = self.inverse_kinematics(end)[1] #end angle
        
        q1s = q1_start + self.s*(q1_end - q1_start) # linear interpolation between start and end 
        q2s = q2_start + self.s*(q2_end - q2_start)# linear interpolation between start and end
        

            
        qs = [q1s, q2s]
        return qs
        
    def plot_end_effector_trajectory(self, qs, increment=0.1,\
                                     plot_q1_against_s=0, plot_q2_against_s=0, plot_q1_against_q2=0):
        """
        Arguments:
            qs - list of parameterised trajectories
            increment - set increment size
            
        return:
            nothing, simply plot a polar map
            
        """
        s = 0
        coordinates = []
        s_axisq1 = []
        s_axisq2 = []
        q1_v_q2 = []
        
        while(s < 1):
            if s == 0:
                coordinates = [(qs[0].subs([(self.s, s)]), qs[1].subs([(self.s, s)]))]
                s_axisq1 = [(s, qs[0].subs([(self.s, s)]))]
                s_axisq2 = [(s, qs[1].subs([(self.s, s)]),s)]
            else:
                coordinates.append((qs[0].subs([(self.s, s)]), qs[1].subs([(self.s, s)])))
                s_axisq1.append((s, qs[0].subs([(self.s, s)])))
                s_axisq2.append((s, qs[1].subs([(self.s, s)])))
            s = s + increment
            
        #print(coordinates)
        
        if plot_q1_against_s == 1:
            simple_plot(s_axisq1, 's', 'q1', 1)
        if plot_q2_against_s == 1:
            simple_plot(s_axisq2, 's', 'q2', 1)        
        if plot_q1_against_q2 == 1:
            simple_plot(coordinates, 'x', 'y', 1)