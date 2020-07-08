# -*- coding: utf-8 -*-
"""
@author: Ryan
"""
import csv
import pickle

def export_tuple_list(data, col_headings, folder, file_name):
    """
    function to take in tuple data and save it to a csv file
    """    

    csvfile=open("saved_data/" + folder +file_name + ".csv",'w', newline='')
    obj=csv.writer(csvfile)
    obj.writerow(col_headings)
    for row in data:
        obj.writerow(row)
    csvfile.close()
    print(file_name + " saved")


def save_obj(obj, folder, name):
    with open('saved_data/pickle_data/' + folder + name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)
        
def load_obj(folder, name):
    with open('saved_data/pickle_data/' + folder + name + '.pkl', 'rb') as f:
        return pickle.load(f)