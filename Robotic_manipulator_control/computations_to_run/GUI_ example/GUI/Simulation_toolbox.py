
import sys,os
current_file_path = os.path.dirname(__file__) 
os.chdir(current_file_path)
sys.path.append('../../../../My_modules/my_basic_modules')
sys.path.append('../../../../My_modules/my_control_modules')
sys.path.append('../../../Robotic_manipulator_control')
sys.path.append(os.path.realpath('../../..'))
sys.path.append(os.path.realpath('../'))


from sympy import symbols, sin, cos
import tkinter as tk
import run_simulation_code
import run_controller_code
import json

from GUI_input_data import input_data
from configure_steps_to_run_page import configure_steps_to_run
from plotting_page import plotting

class main_application:
    def __init__(self, master, input_data):
        self.master = master
        master.title("Simulation Toolbox")

        self.data = input_data
        
        self.width = 300
        self.height = 400

        self.add_menubar()
        #create frames
        self.sim_setup_frame = tk.LabelFrame(self.master, text="Simulation Parameters", width=250, height=200,  borderwidth = 1, relief = tk.SUNKEN)
        self.default_simulation_parameters_frame =  tk.LabelFrame(self.master, text="default Parameters", width=250, height=50,  borderwidth = 1, relief = tk.SUNKEN)
        self.step_to_run_frame = tk.LabelFrame(self.master, text="Steps of simulation to run", width=250, height=100,  borderwidth = 1, relief = tk.SUNKEN)
        self.main_button_frame = tk.Frame(self.master, width=250, height=200,  borderwidth = 1, relief = tk.SUNKEN)        
        
        self.default_controller_frame = tk.LabelFrame(self.master, text="default controller parameters", width=250, height=100,  borderwidth = 1, relief = tk.SUNKEN)
        self.controller_setup_frame = tk.LabelFrame(self.master, text="controller setup", width=250, height=100,  borderwidth = 1, relief = tk.SUNKEN)
        self.run_controller_button_frame = tk.Frame(self.master, width=250, height=200,  borderwidth = 1, relief = tk.SUNKEN)
        #self.step_to_run_frame.grid_propagate(False)
        
        #populate all frames
        self.populate_default_simulation_parameters_frame()
        self.populate_steps_to_run_frame()
        self.populate_simulation_parameters_frame()
        self.populate_run_button_frame()
        self.populate_default_controller_frame()
        self.populate_controller_setup_frame()
        self.populate_run_controller_button_frame()
        #place all frames
        self.default_simulation_parameters_frame.grid(row=0, column = 0, columnspan=2,padx=20, pady=20)
        self.sim_setup_frame.grid(row=1, column = 0, padx=10, pady=10)#, expand=True, fill=tk.BOTH)#, expand=True, fill=tk.BOTH)
        
        self.step_to_run_frame.grid(row=2, column = 0, padx=5, pady=10)        
        self.main_button_frame.grid(row=3, column = 0, padx=10, pady=10)
        self.default_controller_frame.grid(row=4, column = 0, padx=10, pady=10)
        self.controller_setup_frame.grid(row=5, column=0, padx=10, pady=10)
        self.run_controller_button_frame.grid(row=6, column=0, padx=10, pady=10)

        #self.master.after(1000, self.poll)
        
    #def poll(self):
        #print(self.data.default_situation)
        #"""" poll and do stuff"""
        #self.master.after(1000, self.poll) 
        
    def populate_default_simulation_parameters_frame(self):
        
        self.use_custom_input_bool = tk.BooleanVar()
        c1 = tk.Checkbutton(self.default_simulation_parameters_frame, text = "Use Custom input", variable=self.use_custom_input_bool, command=self.use_custom_input)
       
        om_label = tk.Label(self.default_simulation_parameters_frame, text="Default situation:")
        number = self.count_default_simulation_parameters()
        i=0
        option_list = []
        while i < number:
            option_list.append(str(i))
            i=i+1
        self.om_variable = tk.StringVar()
        self.om_variable.set(option_list[0])
    
        self.om_default_parameters = tk.OptionMenu(self.default_simulation_parameters_frame, self.om_variable, *option_list)        
        self.om_default_parameters.grid(row=0, column=1)
        button_width = 100
        button_height = 50
        
        self.data.default_situation = int(self.om_variable.get())
        
        spacer = tk.Label(self.default_simulation_parameters_frame, text="")
        #define button
        add_parameters = tk.Button(self.default_simulation_parameters_frame, text = 'Set parameters', width = 15, command = self.autofill_simulation_parameters)        
        om_label.grid(row=0, column=0)
        c1.grid(row=1, column=0)
        spacer.grid(row=1, column=1)
        add_parameters.grid(row=1, column=2, padx=10, pady=10)

    def populate_simulation_parameters_frame(self):
        #create and place labels for x1 scan
        tk.Label(self.sim_setup_frame, text="x_1 scan properties:").grid(row=0, column=0)
        tk.Label(self.sim_setup_frame, text="start->").grid(row=0, column=1)
        tk.Label(self.sim_setup_frame, text="end->").grid(row=0, column=3)
        tk.Label(self.sim_setup_frame, text="increment->").grid(row=0, column=5)
        #create entry boxes
        self.x1_start = tk.Entry(self.sim_setup_frame, width=4)
        self.x1_end = tk.Entry(self.sim_setup_frame, width=4)
        self.x1_increment = tk.Entry(self.sim_setup_frame, width=4)
        #place entry boxes
        self.x1_start.grid(row=0, column=2)
        self.x1_end.grid(row=0, column=4)
        self.x1_increment.grid(row=0, column=6)
                
        #create and place labels for x_2 scan
        tk.Label(self.sim_setup_frame, text="x_2 scan properties:").grid(row=1, column=0)
        tk.Label(self.sim_setup_frame, text="start->").grid(row=1, column=1)
        tk.Label(self.sim_setup_frame, text="end->").grid(row=1, column=3)
        tk.Label(self.sim_setup_frame, text="increment->").grid(row=1, column=5)
        #create entry boxes
        self.x2_start = tk.Entry(self.sim_setup_frame, width=4)
        self.x2_end = tk.Entry(self.sim_setup_frame, width=4)
        self.x2_increment = tk.Entry(self.sim_setup_frame, width=4)     
        #place entry boxes
        self.x2_start.grid(row=1, column=2)
        self.x2_end.grid(row=1, column=4)
        self.x2_increment.grid(row=1, column=6)
          
        #create and place labels for target set
        tk.Label(self.sim_setup_frame, text="Target set:").grid(row=3, column=0)
        tk.Label(self.sim_setup_frame, text="x_1->").grid(row=3, column=1)
        tk.Label(self.sim_setup_frame, text="x_2 upper->").grid(row=3, column=3)
        tk.Label(self.sim_setup_frame, text="x_2 lower->").grid(row=3, column=5)
        #create entry boxes
        self.x1_target = tk.Entry(self.sim_setup_frame, width=4)
        self.x2_upper = tk.Entry(self.sim_setup_frame, width=4)
        self.x2_lower = tk.Entry(self.sim_setup_frame, width=4) 
        #place entry boxes
        self.x1_target.grid(row=3, column=2)
        self.x2_upper.grid(row=3, column=4)
        self.x2_lower.grid(row=3, column=6)

        #create and place labels for x_2 scan
        tk.Label(self.sim_setup_frame, text="path type:").grid(row=4, column=0)
        tk.Label(self.sim_setup_frame, text="definition->").grid(row=4, column=2)

        #create entry boxes
        self.path_type = tk.Entry(self.sim_setup_frame, width=15)
        self.path_def = tk.Entry(self.sim_setup_frame, width=35)

        #place entry boxes
        self.path_type.grid(row=4, column=1)
        self.path_def.grid(row=4, column=3)
        
        #create and place labels for x_2 scan
        tk.Label(self.sim_setup_frame, text="folder name:").grid(row=5, column=0)
        #create entry boxes
        self.folder_name = tk.Entry(self.sim_setup_frame, width=25)
        #place entry boxes
        self.folder_name.grid(row=5, column=1, columnspan=2)

        #print(type(lower_constraint), lower_constraint)
        
        tk.Label(self.sim_setup_frame, text="additional upper constraint is: 0 = ").grid(row=6, column=0) 
        self.upper_constraint_string = tk.Entry(self.sim_setup_frame, width=100)
        self.upper_constraint_string.grid(row=6, column=1, columnspan=4) 

        tk.Label(self.sim_setup_frame, text="additional lower constraint is: 0 = ").grid(row=7, column=0)
        self.lower_constraint_string = tk.Entry(self.sim_setup_frame, width=100)
        self.lower_constraint_string.grid(row=7,column=1, columnspan=4) 
        
        for child in self.sim_setup_frame.winfo_children():
            child.configure(state='disable')

    def populate_steps_to_run_frame(self):
        #self.add_steps_to_run_checkbuttons()

        w = self.width
        h = self.height
        
        #define checkbox variables
        self.fully_run_step_1= tk.BooleanVar()
        self.fully_run_step_2= tk.BooleanVar()
        self.fully_run_step_3= tk.BooleanVar()
        self.fully_run_step_4= tk.BooleanVar()
        #define checkboxes
        c1 = tk.Checkbutton(self.step_to_run_frame, text = "Step 1", variable=self.fully_run_step_1, command=self.cb_fully_run_step1)
        c2 = tk.Checkbutton(self.step_to_run_frame, text = "Step 2", variable=self.fully_run_step_2, command=self.cb_fully_run_step2)
        c3 = tk.Checkbutton(self.step_to_run_frame, text = "Step 3", variable=self.fully_run_step_3, command=self.cb_fully_run_step3)
        c4 = tk.Checkbutton(self.step_to_run_frame, text = "Step 4", variable=self.fully_run_step_4, command=self.cb_fully_run_step4)
        
        #place checkboxes
        c1.grid(row=0, column=0)
        c2.grid(row=0, column=3)
        c3.grid(row=1, column=0)
        c4.grid(row=1, column=3)
        
        button_width = 100
        button_height = 50
        #define buttons
        badvanced = tk.Button(self.step_to_run_frame, text = 'Advanced', width = 15, command = self.add_configure_sim_to_run_settings_page)        
        #run_simulations.place(x=(w/2) - button_width/2, y=button_width-25, height=button_height, width=button_width)    
        badvanced.grid(row=2, column=3, padx=10, pady=10)
        
    def populate_run_button_frame(self):
        w = self.width
        h = self.height
        
        button_width = 100
        button_height = 50
        #define buttons
        run_simulations = tk.Button(self.main_button_frame, text = 'Run', width = 15, command = self.run_simulation)        
        #run_simulations.place(x=(w/2) - button_width/2, y=button_width-25, height=button_height, width=button_width)    
        run_simulations.grid(row=2, column=0)            


    def populate_default_controller_frame(self):   
        
        self.use_custom_controller_bool = tk.BooleanVar()
        c1 = tk.Checkbutton(self.default_controller_frame, text = "Use Custom input", variable=self.use_custom_controller_bool, command=self.use_custom_controller_input)

        self.time_controller_bool = tk.BooleanVar()
        c2 = tk.Checkbutton(self.default_controller_frame, text = "run_time_experiment", variable=self.time_controller_bool, command=self.add_to_default_controller_frame)
        
        om_label = tk.Label(self.default_controller_frame, text="Default situation:")
        number = self.count_default_simulation_parameters()
                
        number = self.count_default_controller_parameters()
        i=0
        option_list = []
        while i < number:
            control_parameters = self.get_default_controller_parameters(i)
            option_list.append(control_parameters['controller_label'])
            i=i+1

        self.om_controller_type = tk.StringVar()
        self.om_controller_type.set(option_list[0])
    
        self.om_default_controller_parameters = tk.OptionMenu(self.default_controller_frame, self.om_controller_type, *option_list)        
        self.om_default_controller_parameters.grid(row=0, column=10)
        
        button_width = 100
        button_height = 50
        
        self.data.controller_to_simulate = self.om_controller_type.get()
        
        spacer = tk.Label(self.default_simulation_parameters_frame, text="")
        #define button
        add_parameters = tk.Button(self.default_controller_frame, text = 'Set parameters', width = 15, command = self.autofill_controller_parameters)   
        om_label.grid(row=0, column=0)
        c1.grid(row=1, column=0)
        c2.grid(row=2, column=0)
        spacer.grid(row=1, column=1)
        add_parameters.grid(row=4, column=2, padx=10, pady=10)

    def populate_controller_setup_frame(self):
        #create and place labels for x1 scan
        tk.Label(self.controller_setup_frame, text="Initial state").grid(row=0, column=0)
        tk.Label(self.controller_setup_frame, text="x_1->").grid(row=0, column=1)
        tk.Label(self.controller_setup_frame, text="x_2->").grid(row=0, column=3)
        tk.Label(self.controller_setup_frame, text="step size increment->").grid(row=0, column=5)
        
        tk.Label(self.controller_setup_frame, text="control guides").grid(row=1, column=0)
        tk.Label(self.controller_setup_frame, text="x2_up = ").grid(row=1, column=1)
        tk.Label(self.controller_setup_frame, text="x2_low = ").grid(row=1, column=5)
                
        #create entry boxes
        self.x1_start_controller = tk.Entry(self.controller_setup_frame, width=4)
        self.x2_start_controller = tk.Entry(self.controller_setup_frame, width=5)
        self.max_step_size_controller = tk.Entry(self.controller_setup_frame, width=4)

        self.upper_guide = tk.Entry(self.controller_setup_frame, width=10)
        self.lower_guide = tk.Entry(self.controller_setup_frame, width=10)


        #place entry boxes
        self.x1_start_controller.grid(row=0, column=2)
        self.x2_start_controller.grid(row=0, column=4)
        self.max_step_size_controller.grid(row=0, column=6)

        self.upper_guide.grid(row=1, column=2)
        self.lower_guide.grid(row=1, column=6)


        for child in self.controller_setup_frame.winfo_children():
            child.configure(state='disable')

    def populate_run_controller_button_frame(self):
        w = self.width
        h = self.height
        
        button_width = 100
        button_height = 50
        #define buttons
        run_simulations = tk.Button(self.run_controller_button_frame, text = 'simulate controller', width = 15, command = self.run_controller_simulation)        
        run_simulations.grid(row=1, column=0)  
        
    def add_to_default_controller_frame(self):
        self.data.performance_test_controller_cb = self.time_controller_bool.get()

        if self.time_controller_bool.get() == True:    
            self.number_of_runs_label = tk.Label(self.default_controller_frame, text="number of runs to average = ")
            self.number_of_runs = tk.Entry(self.default_controller_frame, width=4)
            self.number_of_runs_label.grid(row=3, column=0)
            self.number_of_runs.grid(row=3, column=2)
        else:
            self.number_of_runs_label.destroy()
            self.number_of_runs.destroy()

    def cb_fully_run_step1(self):

        self.data.step1_cb1 = self.fully_run_step_1.get()
        self.data.step1_cb2 = self.fully_run_step_1.get()

    def cb_fully_run_step2(self):

        self.data.step2_cb1 = self.fully_run_step_2.get()
        self.data.step2_cb2 = self.fully_run_step_2.get()
        self.data.step2_cb3 = self.fully_run_step_2.get()
        self.data.step2_cb4 = self.fully_run_step_2.get()
        self.data.step2_cb5 = self.fully_run_step_2.get()

    def cb_fully_run_step3(self):
        pass

    def cb_fully_run_step4(self):
        self.data.step4_cb1 = self.fully_run_step_4.get()
        self.data.step4_cb2 = self.fully_run_step_4.get()
        
    def autofill_simulation_parameters(self):
        
        #allow the entry boxes to be changed
        for child in self.sim_setup_frame.winfo_children():
            child.configure(state='normal')
        
        self.data.default_situation = int(self.om_variable.get())

        print(self.data.default_situation)
        default_simulation_paramters = self.get_default_simulation_parameters(self.data.default_situation)    
        dsp = default_simulation_paramters
             
        try:            
            #set parameters 
            self.x1_start.delete(0, tk.END)
            self.x1_end.delete(0, tk.END)
            self.x1_increment.delete(0, tk.END)
            
            self.x1_start.insert(0,dsp['x1_lim'][0])
            self.x1_end.insert(0,dsp['x1_lim'][1])
            self.x1_increment.insert(0,dsp['x1_lim'][2])
    
            self.x2_start.delete(0, tk.END)
            self.x2_end.delete(0, tk.END)
            self.x2_increment.delete(0, tk.END)
            
            self.x2_start.insert(0,dsp['x2_lim'][0])
            self.x2_end.insert(0,dsp['x2_lim'][1])
            self.x2_increment.insert(0,dsp['x2_lim'][2])
                        
            self.x1_target.delete(0,tk.END)
            self.x2_upper.delete(0,tk.END)
            self.x2_lower.delete(0,tk.END)
            
            self.x1_target.insert(0,dsp['target_set'][0][0])
            self.x2_upper.insert(0,dsp['target_set'][0][1])
            self.x2_lower.insert(0,dsp['target_set'][1][1])
            
            self.path_type.delete(0,tk.END)
            self.path_def.delete(0,tk.END)
            
            #print(dsp['path_definition'][1])
            
            self.path_type.insert(0,dsp['path_definition'][0])
            self.path_def.insert(0,dsp['path_definition'][1])            
            
            self.folder_name.delete(0,tk.END)            
            self.folder_name.insert(0,dsp['folder_name'])   
            
            self.upper_constraint_string.delete(0,tk.END)
            self.upper_constraint_string.insert(0,dsp['additional_upper_constraint'])
            
            self.lower_constraint_string.delete(0,tk.END)
            self.lower_constraint_string.insert(0,dsp['additional_lower_constraint'])
            
            
        except:
            message = tk.messagebox.Message(title= "Invalid input", message="please enter a valid number")       
            message.show()

        
        for child in self.sim_setup_frame.winfo_children():
            child.configure(state='disable')

    def autofill_controller_parameters(self):
        
        #allow the entry boxes to be changed
        for child in self.controller_setup_frame.winfo_children():
            child.configure(state='normal')
        
        #save the contoller type to selected
        self.data.controller_situation = self.om_controller_type.get()
        
        self.data.performance_test_controller_cb = self.time_controller_bool.get()
        
        if self.data.performance_test_controller_cb == True:
            self.data.number_of_runs_to_average = int(self.number_of_runs.get())
        else:
            self.data.number_of_runs_to_average = 0
        
        number = self.count_default_controller_parameters()
        i=0
        while i < number:
            control_parameters = self.get_default_controller_parameters(i)
            if control_parameters['controller_label'] == self.data.controller_situation:
                break
            i=i+1
        
        if control_parameters['control_guide_type'] == "custom" or control_parameters['control_guide_type'] == 'raw':
            self.x1_start_controller.delete(0,tk.END)
            self.x1_start_controller.insert(0,str(control_parameters['initial_state'][0]))
 
            self.x2_start_controller.delete(0,tk.END)
            self.x2_start_controller.insert(0,str(control_parameters['initial_state'][1]))

            self.max_step_size_controller.delete(0,tk.END)
            self.max_step_size_controller.insert(0,str(control_parameters['integration_step']))

            self.upper_guide.delete(0,tk.END)
            self.upper_guide.insert(0,str(control_parameters['upper_guide']))
            
            self.lower_guide.delete(0,tk.END)
            self.lower_guide.insert(0,str(control_parameters['lower_guide']))
        
        
        for child in self.controller_setup_frame.winfo_children():
            child.configure(state='disable')
                
    def use_custom_input(self):
        print(self.use_custom_input_bool.get())
        if self.use_custom_input_bool.get() == True:            
            for child in self.sim_setup_frame.winfo_children():
                child.configure(state='normal')
        elif self.use_custom_input_bool.get() == False:
            for child in self.sim_setup_frame.winfo_children():
                child.configure(state='disable')


    def use_custom_controller_input(self):
        print(self.use_custom_controller_bool.get())
        if self.use_custom_controller_bool.get() == True:            
            for child in self.controller_setup_frame.winfo_children():
                child.configure(state='normal')
        elif self.use_custom_controller_bool.get() == False:
            for child in self.controller_setup_frame.winfo_children():
                child.configure(state='disable')

    def add_configure_sim_to_run_settings_page(self):

        self.newWindow = tk.Toplevel(self.master)
        app = configure_steps_to_run(self.newWindow, self.data)
 
    def add_plotting_page(self):
        
        self.newWindow = tk.Toplevel(self.master)
        self.app = plotting(self.newWindow, self.data)
            
    def add_menubar(self):
        #Menubar
        menubar = tk.Menu(self.master)
        self.master.config(menu=menubar)
        #Frames
        #ADDEmployeeFrame = tk.Frame(self.master)
        #REMOVE_EmployeeFrame = tk.Frame(self.master)
        
        ##Add Cascade
        File_menu = tk.Menu(menubar, tearoff=0)
        File_menu.add_command(label="Simulation Plots",command=self.add_plotting_page)
        menubar.add_cascade(label="Plotting", menu=File_menu)

    def count_default_simulation_parameters(self):
        with open('default_simulation_parameters.txt') as json_file:
            data = json.load(json_file)
            simulation_paramters = data['simulation_parameters']

        number_of_defaults_available = len(simulation_paramters)
        return number_of_defaults_available


    def count_default_controller_parameters(self):
        with open('default_control_parameters.txt') as json_file:
            data = json.load(json_file)
            simulation_paramters = data['control_parameters']

        number_of_defaults_available = len(simulation_paramters)
        return number_of_defaults_available


    def get_default_simulation_parameters(self, default_number):
        
        with open('default_simulation_parameters.txt') as json_file:
            data = json.load(json_file)
            simulation_parameters = data['simulation_parameters'][default_number]
            
        return simulation_parameters


    def get_default_controller_parameters(self, default_number):
        
        with open('default_control_parameters.txt') as json_file:
            data = json.load(json_file)
            control_parameters = data['control_parameters'][default_number]
            
        return control_parameters

    def get_things_to_run(self):
              
        run_step_1 = [self.data.step1_cb1, self.data.step1_cb2]
        run_step_2 = [self.data.step2_cb1, self.data.step2_cb2,self.data.step2_cb3,self.data.step2_cb4,self.data.step2_cb5]
        run_step_3 = [self.fully_run_step_3.get()]
        run_step_4 = [self.data.step4_cb1, self.data.step4_cb2]

        things_to_run = {
             'step 1': run_step_1,\
             'step 2': run_step_2,\
             'step 3': run_step_3,\
             'step 4': run_step_4
            }
        
        return things_to_run
        
    def set_simulation_parameters(self):        
                
        robot = {'joint_masses': [0.25, 0.25],\
                 'link_lengths': [0.2,0.2],\
                 'actuator_limits': [(-10,10), (-10,10)]}
        
                    
        self.data.additional_upper_constraint = self.upper_constraint_string.get()
        self.data.additional_lower_constraint = self.lower_constraint_string.get()
            
        self.data.x1_start = float(self.x1_start.get())
        self.data.x1_end = float(self.x1_end.get())
        self.data.x1_increment = float(self.x1_increment.get())
        
        self.data.x2_start = float(self.x2_start.get())
        self.data.x2_end = float(self.x2_end.get())
        self.data.x2_increment = float(self.x1_increment.get())
        
        self.data.path_type = self.path_type.get()
        self.data.path_def = self.path_def.get()
        
        self.data.x1_target = float(self.x1_target.get())
        self.data.x2_upper = float(self.x2_upper.get())
        self.data.x2_lower = float(self.x2_lower.get())
        
        self.data.folder_name = self.folder_name.get()
        
        simulation_parameters = {'robot': robot,\
                                'x1_lim':[self.data.x1_start, self.data.x1_end, self.data.x1_increment],\
                                'x2_lim':  [self.data.x2_start, self.data.x2_end, self.data.x2_increment],\
                                'path_definition': [self.data.path_type, self.data.path_def],\
                                'target_set': [(self.data.x1_target , self.data.x2_upper), (self.data.x1_target , self.data.x2_lower)],\
                                'folder_name': self.data.folder_name}
        
        return simulation_parameters
    
    def run_simulation(self):
       
        steps_to_run = self.get_things_to_run()
        simulation_parameters = self.set_simulation_parameters()

        simulation_object = run_simulation_code.simulation(simulation_parameters, steps_to_run, self.data)
        self.data.simulation_object = simulation_object
        
    def set_controller_parameters(self):        

        self.data.x1_start_controller = float(self.x1_start_controller.get())
        self.data.x2_start_controller = float(self.x2_start_controller.get())
        self.data.max_step_size_controller = float(self.max_step_size_controller.get())

        self.data.upper_guide = self.upper_guide.get()
        self.data.lower_guide = self.lower_guide.get()

    def run_controller_simulation(self):
        "run the controller portion of the simulation or get a timed average of a few runs"
        self.set_controller_parameters()
        simulation_object = self.data.simulation_object
        controller_sim = run_controller_code.controller_simulation(simulation_object, self.data)
        #self.Cqs, self.gqs()
        print(self.data.performance_test_controller_cb)
        if self.data.performance_test_controller_cb == True:
            print(self.data.simulation_object.robot.Mqs)
            #run_controller_code.controller_simulation_performance_tests(controller_sim, self.data)
        else:
            controller_sim.run_state_feedback_controller()
            #controller_sim.test_new_constraint_function()
            self.data.control_object = controller_sim
        
def main():
    global root
    root = tk.Tk()
    data = input_data()
    app = main_application(root, data)

    root.mainloop()
    
if __name__ == '__main__':
    main()