"""
This module will simply hold functions useful for sorting data from one form to another for any purpose
"""
import numpy as np
import matplotlib.pyplot as plt


def combine_to_tuples(list1, list2):
    """
    This function simply takes in 2 lists of any length and creates a list of tuples length 2
    The first elements returned will be of the following form for an N dimensional list 
    [(list1[0], list2[0]), ...,(list1[N], list2[N]) ]
    """
    sublist = []
    
    #display simple message if input lists do
    if len(list1) == len(list2):
        i = 0    
        
        for el in list1:
            if i == 0:
                sublist = [(el, list2[i])]        
            else:
                sublist.append((el, list2[i]))
            i=i+1
    else:
        raise Exception("Input lists not of equal length")
   
    
    return sublist

def order_states_by_x1(states_to_be_ordered, supporting_list=[]):
    """
    the input is a list of tuples decribing states like
    [(x1, x2), ...,(x1[N], x2[N]) ]
    len(states_to_be_ordered) must be at least 2
    """
    
    if len(states_to_be_ordered) > 1:
        
        #x1= [x[0] for x in states_to_be_ordered]
        i = 0
        state_shift_counter = 1 # set to 1 to start
        

        while i < len(states_to_be_ordered)-1:
            
            x1_curr = states_to_be_ordered[i][0]
            x1_next = states_to_be_ordered[i+1][0]
            #print(x1_curr, x1_next)
            """
            #if the current x1_value is greater than the next one 
            swap them
            """
            
            if x1_curr > x1_next:

                states_to_be_ordered[i+1], states_to_be_ordered[i] = \
                    states_to_be_ordered[i], states_to_be_ordered[i+1]
                
                """
                sometimes a second list will be provided that will be reordered
                identically to the first
                """
                if len(supporting_list) == len(states_to_be_ordered):
                    supporting_list[i+1], supporting_list[i] = \
                    supporting_list[i], supporting_list[i+1] 
                #start again if a swap is made
                i = -1
                
            i=i+1
        
        if len(supporting_list) == len(states_to_be_ordered):
            return states_to_be_ordered, supporting_list         
        else:
            return states_to_be_ordered
            
    else:
        print("cannot sort a list if its a single state")
        if len(supporting_list) == len(states_to_be_ordered):
            return states_to_be_ordered, supporting_list 
        else:
            return states_to_be_ordered
    
if __name__ == "__main__": 
    
    states = [ (1,5),(15,5), (0.75,1), (0.255, 1),  (0.53,1), (0.245, 1)]
    sup_list = [1,2,3,4,5,6]
    ordered_states, s = order_states_by_x1(states, supporting_list=sup_list)
    print("ordered_states", ordered_states)
    print("s", s)    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    