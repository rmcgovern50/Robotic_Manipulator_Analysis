import tkinter as tk
import run_code

import json

class input_data:
    def __init__(self):
        pass
    
    def set_use_custom_input_bool(self, use_custom_input_bool):
        self.use_custom_input_bool = use_custom_input_bool 
    def get_use_custom_input_bool(self):
        return self.use_custom_input_bool
    
    def set_x1_start(self, x1_start):
        self.x1_start = x1_start
    def get_x1_start(self):
        return self.x1_start
    
    
    

class main_application:
    def __init__(self, master, input_data):
        self.master = master
        master.title("Simulation Toolbox")
        
        self.data = input_data
        
        self.data.set_x1_start(10)
        
        print(self.data.get_x1_start())
        
        self.width = 300
        self.height = 400
        #master.geometry(str(self.width) +"x" + str(self.height)) 
        self.add_menubar()
        #create frames
        self.sim_setup_frame = tk.LabelFrame(self.master, text="Simulation Parameters", width=250, height=200,  borderwidth = 1, relief = tk.SUNKEN)
        self.default_simulation_parameters_frame =  tk.LabelFrame(self.master, text="default Parameters", width=250, height=50,  borderwidth = 1, relief = tk.SUNKEN)
        self.step_to_run_frame = tk.LabelFrame(self.master, text="Steps of simulation to run", width=250, height=100,  borderwidth = 1, relief = tk.SUNKEN)
        self.main_button_frame = tk.Frame(self.master, width=250, height=200,  borderwidth = 1, relief = tk.SUNKEN)        

        #self.step_to_run_frame.grid_propagate(False)
        
        #populate all frames
        self.populate_default_simulation_parameters_frame()
        self.populate_steps_to_run_frame()
        self.populate_simulation_parameters_frame()
        self.populate_run_button_frame()
        
        #place all frames
        self.default_simulation_parameters_frame.grid(row=0, column = 0, columnspan=2,padx=20, pady=20)
        self.sim_setup_frame.grid(row=1, column = 0, padx=10, pady=10)#, expand=True, fill=tk.BOTH)#, expand=True, fill=tk.BOTH)
        
        self.step_to_run_frame.grid(row=2, column = 0, padx=5, pady=10)        
        self.main_button_frame.grid(row=3, column = 0, padx=10, pady=10)
 
    def populate_default_simulation_parameters_frame(self):
        
        self.use_custom_input_bool = tk.BooleanVar()
        c1 = tk.Checkbutton(self.default_simulation_parameters_frame, text = "Use Custom input", variable=self.use_custom_input_bool, command=self.use_custom_input)
        c1.grid(row=0, column=0)
        
        #create and place labels
        tk.Label(self.default_simulation_parameters_frame, text="Default situation:").grid(row=1, column=0)
        #create entry boxes
        self.situation = tk.Entry(self.default_simulation_parameters_frame, width=4)
        #place entry boxes
        self.situation.grid(row=1, column=1)

        button_width = 100
        button_height = 50
        #define buttons
        add_parameters = tk.Button(self.default_simulation_parameters_frame, text = 'Set parameters', width = 15, command = self.autofill_simulation_parameters)        
        #run_simulations.place(x=(w/2) - button_width/2, y=button_width-25, height=button_height, width=button_width)
        add_parameters.grid(row=2, column=0)

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
        c1 = tk.Checkbutton(self.step_to_run_frame, text = "Step 1", variable=self.fully_run_step_1)
        c2 = tk.Checkbutton(self.step_to_run_frame, text = "Step 2", variable=self.fully_run_step_2)
        c3 = tk.Checkbutton(self.step_to_run_frame, text = "Step 3", variable=self.fully_run_step_3)
        c4 = tk.Checkbutton(self.step_to_run_frame, text = "Step 4", variable=self.fully_run_step_4)
        
        #place checkboxes
        c1.grid(row=0, column=0)
        c2.grid(row=0, column=3)
        c3.grid(row=1, column=0)
        c4.grid(row=1, column=3)
        
        button_width = 100
        button_height = 50
        #define buttons
        run_simulations = tk.Button(self.step_to_run_frame, text = 'Advanced', width = 15, command = self.add_configure_sim_to_run_settings_page)        
        #run_simulations.place(x=(w/2) - button_width/2, y=button_width-25, height=button_height, width=button_width)    
        run_simulations.grid(row=2, column=3)
        
    def populate_run_button_frame(self):
        w = self.width
        h = self.height
        
        button_width = 100
        button_height = 50
        #define buttons
        run_simulations = tk.Button(self.main_button_frame, text = 'Run', width = 15, command = self.run_simulation)        
        #run_simulations.place(x=(w/2) - button_width/2, y=button_width-25, height=button_height, width=button_width)    
        run_simulations.grid(row=2, column=0)            

    def autofill_simulation_parameters(self):
        
        #allow the entry boxes to be changed
        for child in self.sim_setup_frame.winfo_children():
            child.configure(state='normal')
        
        default_simulation = int(self.situation.get())
        try: 
            default_simulation_paramters = self.get_default_simulation_parameters(default_simulation)    
            dsp = default_simulation_paramters
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
            
        except:
            message = tk.messagebox.Message(title= "Invalid input", message="please enter a valid number")       
            message.show()

        
        for child in self.sim_setup_frame.winfo_children():
            child.configure(state='disable')


    def use_custom_input(self):
        print(self.use_custom_input_bool.get())
        if self.use_custom_input_bool.get() == True:            
            for child in self.sim_setup_frame.winfo_children():
                child.configure(state='normal')
        elif self.use_custom_input_bool.get() == False:
            for child in self.sim_setup_frame.winfo_children():
                child.configure(state='disable')


    def add_configure_sim_to_run_settings_page(self):

        self.newWindow = tk.Toplevel(self.master)
        app = configure_steps_to_run(self.newWindow, self.data)
        
        print("data is ", app.testvar.get())
        
        
        
    def add_plotting_page(self):
        
        self.newWindow = tk.Toplevel(self.master)
        self.app = plotting(self.newWindow)
            
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


    def get_default_simulation_parameters(self, situation):

        with open('default_simulation_parameters.txt') as json_file:
            data = json.load(json_file)
            simulation_paramters = data['simulation_parameters'][situation]
            
        return simulation_paramters


    def get_things_to_run(self):
        
        #define checkbox variables
        if self.fully_run_step_1.get():
            run_step_1 = True
        else:
            run_step_1 = False
            
        if self.fully_run_step_2.get():
            run_step_2 = True
        else:
            run_step_2 = False
            
        if self.fully_run_step_3.get():
            run_step_3 = True
        else:
            run_step_3 = False
            
        if self.fully_run_step_4.get():
            run_step_4 = True
        else:
            run_step_4 = False    
        
        things_to_run = {
             'step 1': [run_step_1],\
             'step 2': [run_step_2],\
             'step 3': [run_step_3],\
             'step 4': [run_step_4],\
            }
        
        return things_to_run

        
    def set_simulation_parameters(self):        
                
        robot = {'joint_masses': [0.25, 0.25],\
                 'link_lengths': [0.2,0.2],\
                 'actuator_limits': [(-10,10), (-10,10)]}


        path_def_list= self.path_def.get()
        
        
        path_def = self.convert_string_back_to_list(path_def_list)
        
        simulation_parameters = {'robot': robot,\
                                'x1_lim':[float(self.x1_start.get()), float(self.x1_end.get()), float(self.x1_increment.get())],\
                                'x2_lim':  [float(self.x1_start.get()), float(self.x1_end.get()), float(self.x1_increment.get())],\
                                'path_definition': [self.path_type.get(), path_def],\
                                'target_set': [ (float(self.x1_target.get()), float(self.x2_upper.get())), (float(self.x1_target.get()), float(self.x2_lower.get()))],\
                                'folder_name': self.folder_name.get()}
                                

        return simulation_parameters
    
    
    def convert_string_back_to_list(self, string):
        #takes a list of tuples containing numerical numbers that has been turned into a string
        #and remakes the orignal list
        list_of_elements = string.split()
        list_length = len(list_of_elements)
        i=0
        #print(list_of_elements)
        collected_parts = ["Blank_element"]
        part_number = 0
        while i < list_length:
            el = list_of_elements[i]
            
            #if we have an open bracket
            if '{' in el:
                new_number_list = [float(el[1:])]
                i = i + 1
                while True:
                    el = list_of_elements[i]
                    
                    if '}' in el:
                        new_number_list.append(float(el[:-1]))
                        break    
                    
                    if ' ' in el:
                        pass
                    else:
                        new_number_list.append(float(el))
                    i = i + 1       
                collected_parts.append(new_number_list)
            elif not(' ' in el):
                #print(el)
                #if we have a single number
                collected_parts.append(float(el))
                
            i= i + 1
        
        collected_parts.pop(0)
        i=0
        for part in collected_parts:
            if type(part) != list:
                tuple_version = part
            else:
                tuple_version = tuple(part)
            collected_parts[i] = tuple_version 
            i=i+1
        
        remade_list = collected_parts
        return remade_list

    
    
    def run_simulation(self):
        #label=tk.Label(self.master, text=str(self.fully_run_step_1.get()) + str(self.fully_run_step_2.get())+ str(self.fully_run_step_3.get())+ str(self.fully_run_step_4.get()))
        
        #default_simulation = 1
        #simulation_parameters = self.get_default_simulation_parameters(default_simulation)
                
        things_to_run = self.get_things_to_run()
        simulation_parameters = self.set_simulation_parameters()
        #print(simulation_parameters)
        #print(things_to_run)
        sim = run_code.simulation(simulation_parameters, things_to_run)
        print(self.data.get_x1_start())            
    
class plotting:
    def __init__(self, master):
        self.master = master
        master.geometry("400x300")
        master.title("Plotting")
        self.frame = tk.Frame(self.master)
        self.quitButton = tk.Button(self.frame, text = 'Quit', width = 25, command = self.close_windows)
        self.quitButton.pack()
        self.frame.pack()
    def close_windows(self):
        self.master.destroy()


    
class configure_steps_to_run:
    def __init__(self, master, input_data):
        self.master = master
        master.geometry("400x300")
        master.title("steps_to_run")
        self.data = input_data
        

        self.step_2_frame = tk.LabelFrame(self.master, text="step 2", width=250, height=200,  borderwidth = 1, relief = tk.SUNKEN)
        
        self.testvar= tk.BooleanVar()
        #define checkboxes
        c1 = tk.Checkbutton(self.step_2_frame, text = "Step 1", variable=self.testvar)
        c1.grid(row=0, column=0)
        
        self.apply_button = tk.Button(self.step_2_frame, text = 'Apply', width = 25, command = self.close_windows)
        self.apply_button.grid(row=1, column=0)
        self.step_2_frame.grid(row=0, column=0)
        
    def close_windows(self):
        
        self.data.set_x1_start(self.testvar.get())
        self.master.destroy()
        


def main(): 
    global root 
    
    root = tk.Tk()
    data = input_data()
    app = main_application(root, data)
    root.mainloop()

if __name__ == '__main__':
    main()
"""
root= GUI.Tk()
root.geometry("400x400")

def Run():
    text_to_print = "Hello"
    label=GUI.Label(root, text=str(fully_run_step_1.get()) + str(fully_run_step_2.get())+ str(fully_run_step_3.get())+ str(fully_run_step_4.get()))
    label.grid(column=lastcol,row=lastrow+1)

Title = root.title("Robot Analysis Toolbox")

lastrow = 5
lastcol = 5

#define checkbox variables
fully_run_step_1= GUI.BooleanVar()
fully_run_step_2= GUI.BooleanVar()
fully_run_step_3= GUI.BooleanVar()
fully_run_step_4= GUI.BooleanVar()
#define checkboxes
c = GUI.Checkbutton(root, text = "Step 1", variable=fully_run_step_1)
c2 = GUI.Checkbutton(root, text = "Step 2", variable=fully_run_step_2)
c3 = GUI.Checkbutton(root, text = "Step 3", variable=fully_run_step_3)
c4 = GUI.Checkbutton(root, text = "Step 4", variable=fully_run_step_4)
#place checkboxes
c.grid(column=0,row=1)
c2.grid(column=0,row=2)
c3.grid(column=1,row=1)
c4.grid(column=1,row=2)

#define push button and place
run_button = GUI.Button(root, text="Run Setup_simulation", command=Run)
run_button.grid(column=lastcol,row=lastrow)
"""



#root.mainloop()