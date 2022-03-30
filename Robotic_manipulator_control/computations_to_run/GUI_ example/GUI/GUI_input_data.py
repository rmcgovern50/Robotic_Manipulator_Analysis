import sympy
from sympy import symbols


class input_data:

    def __init__(self):
        
        self.initialise_all_checkbox_variables()
        
        self.additional_upper_constraint = "N/A"
        self.additional_lower_constraint = "N/A"        

    def initialise_all_checkbox_variables(self):
        self.step1_cb1 = False
        self.step1_cb2 = True

        self.step2_cb1 = False
        self.step2_cb2 = False
        self.step2_cb3 = False
        self.step2_cb4 = False
        self.step2_cb5 = False
        
        self.step4_cb1 = False        
        self.step4_cb2 = False   
        self.simulate_controller = False
        
        self.use_matlab_cb = False

        self.performance_test_controller_cb = False
        
    @property
    def default_situation(self):
        return self._default_situation
    @default_situation.setter
    def default_situation(self, value):
        self._default_situation = value
        

    @property
    def controller_situation(self):
        return self._controller_situation
    @controller_situation.setter
    def controller_situation(self, value):
        self._controller_situation = value    
    
    @property
    def x1_start(self):
        return self._x1_start
    @x1_start.setter
    def x1_start(self, value):
        self._x1_start = value
    
    
    @property
    def x1_end(self):
        return self._x1_end
    @x1_end.setter
    def x1_end(self, value):
        self._x1_end = value


    @property
    def x1_increment(self):
        return self._x1_increment
    @x1_increment.setter
    def x1_increment(self, value):
        self._x1_increment = value


    @property
    def x2_start(self):
        return self._x2_start
    @x2_start.setter
    def x2_start(self, value):
        self._x2_start = value


    @property
    def x2_end(self):
        return self._x2_end
    @x2_end.setter
    def x2_end(self, value):
        self._x2_end = value


    @property
    def x2_increment(self):
        return self._x2_increment
    @x2_increment.setter
    def x2_increment(self, value):
        self._x2_increment = value
    

    @property
    def path_type(self):
        return self._path_type
    @path_type.setter
    def path_type(self, value):
        self._path_type = value
        

    @property
    def path_def(self):
        return self._path_def
    @path_def.setter
    def path_def(self, value):
        destringafied_value = self.convert_string_back_to_list(value)
        self._path_def = destringafied_value



    @property
    def step1_cb1(self):
        return self._step1_cb1
    @step1_cb1.setter
    def step1_cb1(self, value):
        self._step1_cb1 = value


    @property
    def step1_cb2(self):
        return self._step1_cb2
    @step1_cb2.setter
    def step1_cb2(self, value):
        self._step1_cb2 = value


    @property
    def step2_cb1(self):
        return self._step2_cb1
    @step2_cb1.setter
    def step2_cb1(self, value):
        self._step2_cb1 = value


    @property
    def step2_cb2(self):
        return self._step2_cb2
    @step2_cb2.setter
    def step2_cb2(self, value):
        self._step2_cb2 = value


    @property
    def step2_cb3(self):
        return self._step2_cb3
    @step2_cb3.setter
    def step2_cb3(self, value):
        self._step2_cb3 = value
        
        
    @property
    def step2_cb4(self):
        return self._step2_cb4
    @step2_cb3.setter
    def step2_cb4(self, value):
        self._step2_cb4 = value
        
        
    @property
    def step2_cb5(self):
        return self._step2_cb5
    @step2_cb3.setter
    def step2_cb5(self, value):
        self._step2_cb5 = value        
        
        
    @property
    def step4_cb1(self):
        return self._step4_cb1
    @step4_cb1.setter
    def step4_cb1(self, value):
        self._step4_cb1 = value
        
        
    @property
    def step4_cb2(self):
        return self._step4_cb2
    @step4_cb2.setter
    def step4_cb2(self, value):
        self._step4_cb2 = value
 

    @property
    def use_matlab_cb(self):
        return self._use_matlab_cb
    @use_matlab_cb.setter
    def use_matlab_cb(self, value):
        self._use_matlab_cb = value
        
    @property
    def performance_test_controller_cb(self):
        return self._performance_test_controller_cb
    @performance_test_controller_cb.setter
    def performance_test_controller_cb(self, value):
        self._performance_test_controller_cb = value 
        
        
    @property
    def simulation_object(self):
        return self._simulation_object
    @simulation_object.setter
    def simulation_object(self, value):
        self._simulation_object = value
     
        
    @property
    def control_object(self):
        return self._control_object
    @control_object.setter
    def control_object(self, value):
        self._control_object = value
        
        
    @property
    def simulate_controller(self):
        return self._simulate_controller
    @simulate_controller.setter
    def simulate_controller(self, value):
        self._simulate_controller = value
    
    
    @property
    def controller_to_simulate(self):
        return self._controller_to_simulate
    @controller_to_simulate.setter
    def controller_to_simulate(self, value):
        self._controller_to_simulate = value
        
        
    @property
    def additional_upper_constraint(self):
        return self._additional_upper_constraint
    @additional_upper_constraint.setter
    def additional_upper_constraint(self, value):
        x1, x2\
        = symbols('x1 x2')
        if value == "N/A":
            value = "N/A"
        else:    
            value = sympy.simplify(value)
        
        self._additional_upper_constraint = value
        
        
    @property
    def additional_lower_constraint(self):
        return self._additional_lower_constraint
    @additional_lower_constraint.setter
    def additional_lower_constraint(self, value):
        x1, x2\
        = symbols('x1 x2')
        if value == "N/A":
            value = "N/A"
        else:    
            value = sympy.simplify(value)
        
        self._additional_lower_constraint = value        


    @property
    def number_of_runs_to_average(self):
        return self._number_of_runs_to_average
    @number_of_runs_to_average.setter
    def number_of_runs_to_average(self, value):
        self._number_of_runs_to_average = value

    @property
    def x1_start_controller(self):
        return self._x1_start_controller
    @x1_start_controller.setter
    def x1_start_controller(self, value):
        self._x1_start_controller = value

    @property
    def x2_start_controller(self):
        return self._x2_start_controller
    @x2_start_controller.setter
    def x2_start_controller(self, value):
        self._x2_start_controller = value


    @property
    def max_step_size_controller(self):
        return self._max_step_size_controller
    @max_step_size_controller.setter
    def max_step_size_controller(self, value):
        self._max_step_size_controller = value


    @property
    def upper_guide(self):
        return self._upper_guide
    @upper_guide.setter
    def upper_guide(self, value):
        x1, x2\
        = symbols('x1 x2')
        if value == "N/A":
            value = "N/A"
        else:    
            value = sympy.simplify(value)
        
        self._upper_guide = value
        
        
    @property
    def lower_guide(self):
        return self._lower_guide
    @lower_guide.setter
    def lower_guide(self, value):
        x1, x2\
        = symbols('x1 x2')
        if value == "N/A":
            value = "N/A"
        else:    
            value = sympy.simplify(value)
        
        self._lower_guide = value        


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