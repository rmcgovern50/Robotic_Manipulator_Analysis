"""
This module will simply hold functions useful for sorting data from one form to another for any purpose
"""


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


#enable to test anything
if __name__ == "__main__": 
   """
    a = [1 , 2 , 3]
   b = ["libeb2", "!"," "]
   print(combine_to_tuples(a, b))
   """
