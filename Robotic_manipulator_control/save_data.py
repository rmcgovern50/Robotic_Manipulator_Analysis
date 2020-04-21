# -*- coding: utf-8 -*-
"""
@author: Ryan
"""
import csv


def export_tuple_list(data, col_headings, file_name):
    """
    function to take in tuple data and save it to a csv file
    """    

    csvfile=open("saved_data/" + file_name + ".csv",'w', newline='')
    obj=csv.writer(csvfile)
    obj.writerow(col_headings)
    for row in data:
        obj.writerow(row)
    csvfile.close()
    print(file_name + " saved")
