import tkinter as tk
from tkinter import messagebox

class configure_steps_to_run:
    def __init__(self, master, input_data):
        self.master = master
        #master.geometry("400x300")
        master.title("steps_to_run")
        self.data = input_data

        self.apply_frame = tk.Frame(self.master)
        
        self.step_1_frame = tk.LabelFrame(self.master, text="step 1", width=250, height=200,  borderwidth = 1, relief = tk.SUNKEN)
        self.step_2_frame = tk.LabelFrame(self.master, text="step 2", width=250, height=200,  borderwidth = 1, relief = tk.SUNKEN)
        self.step_3_frame = tk.LabelFrame(self.master, text="step 3", width=250, height=200,  borderwidth = 1, relief = tk.SUNKEN)
        self.step_4_frame = tk.LabelFrame(self.master, text="step 4", width=250, height=200,  borderwidth = 1, relief = tk.SUNKEN)
        
        self.step_1_frame.grid_propagate(False)
        self.step_2_frame.grid_propagate(False)
        self.step_3_frame.grid_propagate(False)
        self.step_4_frame.grid_propagate(False)
        
        
        self.populate_checkbuttons_step_1()
        self.populate_checkbuttons_step_2()
        self.populate_checkbuttons_step_3()
        self.populate_checkbuttons_step_4()
        
        self.populate_button_frame()
        
        self.step_1_frame.grid(row=0, column=0)
        self.step_2_frame.grid(row=0, column=1)
        self.step_3_frame.grid(row=0, column=2)
        self.step_4_frame.grid(row=0, column=3)
        
        self.apply_frame.grid(row=1, column=3)        
        
        
    def populate_checkbuttons_step_1(self):

        self.construct_manipulator = tk.BooleanVar()
        self.create_control_object = tk.BooleanVar()

        cb1 = tk.Checkbutton(self.step_1_frame, text = "create manipulator object", variable=self.construct_manipulator)
        cb2 = tk.Checkbutton(self.step_1_frame, text = "create control object", variable=self.create_control_object)   
      
        cb1.grid(row=0, column=0, sticky="w")    
        cb2.grid(row=1, column=0, sticky="w")
        
    def populate_checkbuttons_step_2(self):
        
        self.run_constraint_simulations = tk.BooleanVar()
        self.run_curve_simulations = tk.BooleanVar()
        self.run_ROS_boundary_xd_sim = tk.BooleanVar()
        self.run_ROS_boundary_xa_sim = tk.BooleanVar()
        self.work_out_steps_to_run = tk.BooleanVar()
        
        cb1 = tk.Checkbutton(self.step_2_frame, text = "run_constraint_simulations", variable=self.run_constraint_simulations)
        cb2 = tk.Checkbutton(self.step_2_frame, text = "run_curve_simulations", variable=self.run_curve_simulations)        
        cb3 = tk.Checkbutton(self.step_2_frame, text = "run_ROS_boundary_xd_sim", variable=self.run_ROS_boundary_xd_sim)
        cb4 = tk.Checkbutton(self.step_2_frame, text = "run_ROS_boundary_xa_sim", variable=self.run_ROS_boundary_xa_sim)        
        cb5 = tk.Checkbutton(self.step_2_frame, text = "work_out_steps_to_run", variable=self.work_out_steps_to_run)        
        
        
        cb1.grid(row=0, column=0, sticky="w")    
        cb2.grid(row=1, column=0, sticky="w")  
        cb3.grid(row=2, column=0, sticky="w")  
        cb4.grid(row=3, column=0, sticky="w")  
        cb5.grid(row=4, column=0, sticky="w")  


    def populate_checkbuttons_step_3(self):
        pass


    def populate_checkbuttons_step_4(self):
        
        self.run_boundary_sim_setup = tk.BooleanVar()
        self.run_boundary_sim = tk.BooleanVar()
        
        cb1 = tk.Checkbutton(self.step_4_frame, text = "run_boundary_sim_setup", variable=self.run_boundary_sim_setup)
        cb2 = tk.Checkbutton(self.step_4_frame, text = "run_boundary_sim", variable=self.run_boundary_sim)        
     
        cb1.grid(row=0, column=0, sticky="w")    
        cb2.grid(row=1, column=0, sticky="w") 

    def populate_button_frame(self):

        self.apply_button = tk.Button(self.apply_frame, text = 'Apply', width = 25, command = self.close_windows)
        self.apply_button.grid(row=0, column=0, padx=10, pady=10)


    def close_windows(self):

        self.data.step1_cb1 = self.construct_manipulator.get()
        self.data.step1_cb2 = self.create_control_object.get() 

        self.data.step2_cb1 = self.run_constraint_simulations.get()
        self.data.step2_cb2 = self.run_curve_simulations.get()
        self.data.step2_cb3 = self.run_ROS_boundary_xd_sim.get()
        self.data.step2_cb4 = self.run_ROS_boundary_xa_sim.get()
        self.data.step2_cb5 = self.work_out_steps_to_run.get()
        
        self.data.step4_cb1 = self.run_boundary_sim_setup.get()
        self.data.step4_cb2 = self.run_boundary_sim.get()
       
        message = tk.messagebox.Message(title= "Information", message="Settings applied")       
        message.show()        
        self.master.destroy()
        
