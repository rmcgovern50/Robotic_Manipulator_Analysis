import tkinter as tk


class plotting:
    def __init__(self, master, input_data):
        self.master = master
        self.data = input_data
        #master.geometry("400x300")
        master.title("Plotting")
        self.plot_selected_frame = tk.Frame(self.master)

        self.simulation_plots_frame = tk.LabelFrame(self.master, text="simulation object", width=250, height=400,  borderwidth = 1, relief = tk.SUNKEN)
        self.controller_plots_frame = tk.LabelFrame(self.master, text="controller", width=250, height=400,  borderwidth = 1, relief = tk.SUNKEN)

        self.simulation_plots_frame.grid_propagate(False)
        self.controller_plots_frame.grid_propagate(False)

        self.populate_plot_button_frame()
        self.populate_checkbuttons_simulation_plots_frame()
        self.populate_checkbuttons_controller_plots_frame()
        
        self.simulation_plots_frame.grid(row=0, column=0)
        self.controller_plots_frame.grid(row=0, column=1)

        self.plot_selected_frame.grid(row=1, column=1)        
    
    def populate_plot_button_frame(self):
        self.plot_button = tk.Button(self.plot_selected_frame, text = 'Plot Selected', width = 25, command = self.produce_plots_and_close)
        self.plot_button.grid(row=1, column=1)

    def populate_checkbuttons_simulation_plots_frame(self):
        
        self.plot_admissible = tk.BooleanVar()
        self.plot_vl_curve = tk.BooleanVar()
        self.plot_joint_space = tk.BooleanVar()
        self.plot_workspace = tk.BooleanVar()
        self.plot_resultant_constraint_curves = tk.BooleanVar()
        self.plot_sx1_upper_data = tk.BooleanVar()
        self.plot_sx1_lower_data = tk.BooleanVar()
        self.plot_sx1_upper_data_with_approximation = tk.BooleanVar()
        self.plot_sx1_lower_data_with_approximation = tk.BooleanVar()
        self.plot_ROS_bounds_with_constraints = tk.BooleanVar()
        
        cb1 = tk.Checkbutton(self.simulation_plots_frame, text = "Admissible Region", variable=self.plot_admissible)
        cb2 = tk.Checkbutton(self.simulation_plots_frame, text = "Velocity Limit Curve", variable=self.plot_vl_curve)        
        cb3 = tk.Checkbutton(self.simulation_plots_frame, text = "Joint Space", variable=self.plot_joint_space)
        cb4 = tk.Checkbutton(self.simulation_plots_frame, text = "Workspace", variable=self.plot_workspace)        
        cb5 = tk.Checkbutton(self.simulation_plots_frame, text = "Plot resultant contraint curves", variable=self.plot_resultant_constraint_curves)        
        cb6 = tk.Checkbutton(self.simulation_plots_frame, text = "Plot s(x1) upper data", variable=self.plot_sx1_upper_data)                
        cb7 = tk.Checkbutton(self.simulation_plots_frame, text = "Plot s(x1) lower data", variable=self.plot_sx1_lower_data)   
        cb8 = tk.Checkbutton(self.simulation_plots_frame, text = "Plot s(x1) upper data with approximation", variable=self.plot_sx1_upper_data_with_approximation)             
        cb9 = tk.Checkbutton(self.simulation_plots_frame, text = "Plot s(x1) lower data with approximation", variable=self.plot_sx1_lower_data_with_approximation)
        cb10 = tk.Checkbutton(self.simulation_plots_frame, text = "Plot ROS bounds with constraints shown", variable=self.plot_ROS_bounds_with_constraints)

        
        cb1.grid(row=0, column=0, sticky="w")    
        cb2.grid(row=1, column=0, sticky="w")  
        cb3.grid(row=2, column=0, sticky="w")  
        cb4.grid(row=3, column=0, sticky="w")  
        cb5.grid(row=4, column=0, sticky="w")  
        cb6.grid(row=5, column=0, sticky="w")
        cb7.grid(row=6, column=0, sticky="w")
        cb8.grid(row=7, column=0, sticky="w")
        cb9.grid(row=8, column=0, sticky="w")
        cb10.grid(row=9, column=0, sticky="w")
        
    def populate_checkbuttons_controller_plots_frame(self):
        
        self.plot_control_trajectory = tk.BooleanVar()
        self.blank1= tk.BooleanVar()
        self.blank1= tk.BooleanVar()
        self.blank1= tk.BooleanVar()
        self.blank1= tk.BooleanVar()
        
        cb1 = tk.Checkbutton(self.controller_plots_frame, text = "Control_trajectory", variable=self.plot_control_trajectory)
        cb2 = tk.Checkbutton(self.controller_plots_frame, text = "blank1", variable=self.blank1)        
        cb3 = tk.Checkbutton(self.controller_plots_frame, text = "blank1", variable=self.blank1)
        cb4 = tk.Checkbutton(self.controller_plots_frame, text = "blank1", variable=self.blank1)        
        cb5 = tk.Checkbutton(self.controller_plots_frame, text = "blank1", variable=self.blank1)                
        
        cb1.grid(row=0, column=0, sticky="w")    
        cb2.grid(row=1, column=0, sticky="w")  
        cb3.grid(row=2, column=0, sticky="w")  
        cb4.grid(row=3, column=0, sticky="w")  
        cb5.grid(row=4, column=0, sticky="w")  

    def produce_all_requested_plots(self):
        
        if self.plot_admissible.get():
            self.data.simulation_object.manipulator.plot_admissible_region()

        if self.plot_vl_curve.get():
            self.data.simulation_object.manipulator.plot_velocity_limit_curve()

        if self.plot_joint_space.get():
            self.data.simulation_object.manipulator.plot_joint_space()

        if self.plot_workspace.get():
            self.data.simulation_object.manipulator.plot_workspace_movement()

        if self.plot_resultant_constraint_curves.get():
            self.data.simulation_object.plot_constraints()

        if self.plot_sx1_upper_data.get():
            self.data.simulation_object.plot_sx1(boundary="upper")

        if self.plot_sx1_lower_data.get():
            self.data.simulation_object.plot_sx1(boundary="lower")

        if self.plot_sx1_upper_data_with_approximation.get():
            self.data.simulation_object.plot_polynomial_approximation_sx(boundary="upper")
            
        if self.plot_sx1_lower_data_with_approximation.get():
            self.data.simulation_object.plot_polynomial_approximation_sx(boundary="lower")

        if self.plot_ROS_bounds_with_constraints.get():
            self.data.control_object.plot_ROS_bounds_with_constraints()
            
        if self.plot_control_trajectory.get():
            self.data.control_object.plot_control_trajectory()
            

    def produce_plots_and_close(self):
        self.produce_all_requested_plots()
        print("we have got the object", self.data.simulation_object)
        self.master.destroy()