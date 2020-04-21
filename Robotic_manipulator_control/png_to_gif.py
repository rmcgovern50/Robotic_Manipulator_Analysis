"""
script adapted from:
https://pythonprogramming.altervista.org/png-to-gif/
"""
from PIL import Image
import glob
import os



def compile_gif(offset_path = "robot_motion_construction_images\\", save_offset="", filename="png2gif", frame_duration=300):

    
    # Create the frames
    frames = []


    #offset_path = "\plots\gifs\gif_construction_images"
    imgs = glob.glob(offset_path+"*.png")
    #get the image names
    #get the filenames
    
    names = [os.path.basename(x) for x in imgs]
    #print(names)
    
    names_no_extension = [int(name[:-4]) for name in  names]
    #print(names_no_extension)
    
    i=0
    #sort images into order based on filename
    while i < len(names_no_extension)-1:
        
        if names_no_extension[i] > names_no_extension[i+1]:
            temp_var1 = names_no_extension[i]
            names_no_extension[i] = names_no_extension[i+1]
            names_no_extension[i+1] = temp_var1
            
            temp_var2 = imgs[i]
            imgs[i] = imgs[i+1]
            imgs[i+1] = temp_var2
    
            
            i = -1
            
        i = i + 1
        
    #print(names_no_extension)

    for i in imgs:
        new_frame = Image.open(i)
        frames.append(new_frame)
     
        # Save into a GIF file that loops forever
        frames[0].save(save_offset+filename+".gif", format='GIF',
                       append_images=frames[1:],
                       save_all=True,
                       duration=frame_duration, loop=0)
        
if __name__ == "__main__": 
    compile_gif()