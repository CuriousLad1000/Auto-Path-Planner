import PySimpleGUI as sg
import sys
from ast import literal_eval
import numpy as np


my_input=sys.stdin.read()
my_input = literal_eval(my_input)

def Auto_gui(dat,loaded_manual_offset,TGT_save):
    font_small = ("gothic", 12)
    font_heading = ("gothic", 14)
    selected_mode = 0 #will determine the initial pose of the robot
    man_offset = 0.0 
    cluster_idx = 0
    selected_motion = 0  #default auto
    iterations = 1 #default
    Exit_flag = 0

    desc = ["Default robot pose with camera facing down towards the object",
            "Robot extended to the highest point and camera facing down towards the object.",
            "Robot facing forward and camera at a very low position. Object right in front of the robot.",
            "Robot facing forward and camera at a higher position. Object right in front of the robot.",
            "Robot and camera looking towards LEFT at a LOW position. Object NOT ALIGNED to robot's base. Manual offset required (Horizontal Distance between robot's base and object).",
            "Robot and camera looking towards LEFT at a HIGHER position. Object NOT ALIGNED to robot's base. Manual offset required (Horizontal Distance between robot's base and object).",
            "Robot EXTENDED and camera looking towards LEFT at a LOW position. Object is aligned to robot's base (Horizontal distance is 0).",
            "Robot EXTENDED and camera looking towards LEFT at a HIGHER position. Object is aligned to robot's base (Horizontal distance is 0).",
            "Robot and camera looking towards RIGHT at a LOW position. Object NOT ALIGNED to robot's base. Manual offset required (Horizontal Distance between robot's base and object).",
            "Robot and camera looking towards RIGHT at a HIGHER position. Object NOT ALIGNED to robot's base. Manual offset required (Horizontal Distance between robot's base and object).",
            "Robot EXTENDED and camera looking towards RIGHT at a LOW position. Object is aligned to robot's base (Horizontal distance is 0).",
            "Robot EXTENDED and camera looking towards RIGHT at a HIGHER position. Object is aligned to robot's base (Horizontal distance is 0).",
           ]
    
    while len(desc) < len(dat):
        desc.append("N/A") #add empty descriptions if more imgs added.
    
    
    file_list_column = [[sg.Text("Select a Pose that best describes your case.", font=font_heading),]]
    for i in range(0, int(np.ceil(len(dat)/2)) ):
        
        file_list_column.append( [sg.Button(button_text = "",enable_events = True, 
                                 size = (16, 4),key = 'but'+str(i+i), image_source=dat[i+i], 
                                 image_size = (None, None), image_subsample = 5, )] )
 
        if (i+i+1)<len(dat): #logic to add second column
            file_list_column[i+1].append( sg.Button(button_text = "",enable_events = True, 
                                         size = (16, 4),key = 'but'+str(i+i+1), image_source=dat[i+i+1], 
                                         image_size = (None, None), image_subsample = 5, ))

    
    buttons_layout = [
        [sg.Radio(text = "Auto select motion Path",font=font_small,default = True, enable_events = True, 
                  tooltip ='Program will decide motion Path',key = 'Rad_Auto',
                  auto_size_text = True, group_id=0, pad = ((5, 68), (0, 0))), 
         sg.Text("Enter Horizontal distance between robot's base and the Object:", size=(53, 1)
                            , font=font_small, expand_y=True), sg.Input(default_text = 0.0, 
                         size=(10, 1),key = "manual_offset",tooltip = "(in Meters)",disabled = True )],
        
        [sg.Radio(text = "Motion along X-axis",font=font_small, default = False, enable_events = True, 
                  tooltip ="Robot's motion along the camera's X axis",key = 'Rad_X',
                  auto_size_text = True, group_id=0 ), 
         sg.Text("", size=(43, 1), auto_size_text = True , font=font_small, expand_y=True), 
         sg.Text("Number of Iterations:", size=(19, 1), font=font_small, expand_y=True), 
         sg.Input(default_text = 1, size=(10, 1),key = "iterations",
            tooltip = "Enter the number of times you would like the robot to scan the surface")],
        
        [sg.Radio(text = "Motion along Y-axis", font=font_small, default = False, enable_events = True, 
                  tooltip ="Robot's motion along the camera's Y axis",key = 'Rad_Y',
                  auto_size_text = True, group_id=0, size=(65, 1) ),
         sg.Text("Save Targets?", size=(14, 1), auto_size_text = True , font=font_small, expand_y=True)
                    , sg.Checkbox("", default = TGT_save, key ="TGT_save",pad=((0, 0),(0,5))
                  , tooltip = "Save Generated Targets for later use.")],
        
        [sg.Text("Default cluster index:", size=(19, 1), pad=((30, 0),(0,5)), font=font_small, expand_y=True), 
         sg.Input(default_text = 0, size=(10, 1),key = "cluster_idx",
            tooltip = "Cluster index to choose if there are more than 1 available"), 
         sg.Text("Hide all Previews after first iteration?", size=(31, 1), auto_size_text = True, 
         font=font_small, expand_y=True, pad=((184, 0),(0,5))), 
         sg.Checkbox("", default = True, key ="Hide_prev",pad=((0, 0),(0,5))
                  , tooltip = "Don't show previews. Fully automate scannig.")],
        
         
         [sg.Button(button_text = "Cancel",enable_events = True, tooltip ='First Cluster will be auto-selected', 
                                     size = (10, 2),key = 'Cancel', image_source=None,
                                     pad = ((250, 0), (10, 0)), 
                                     image_size = (None, None), image_subsample = None, ),
         sg.Button(button_text = "OK",enable_events = True, disabled = True, tooltip ='Select atleast one cluster',
                                     size = (10, 2),key = 'OK', image_source=None, pad = ((20, 0), (10, 0)),
                                     image_size = (None, None), image_subsample = None )] ]

   
    image_viewer_column = [
        [sg.Text("Preview", font = font_heading)],
        [sg.Text("Description: " + desc[0],size=(100, 2),font=font_small, key="-TOUT-")],
        [sg.Image(key="img",source=dat[0])], [sg.HSeparator(pad=(0,10))], [sg.Column(buttons_layout)]]

    
    
    layout = [ [ sg.Column(file_list_column,size = (None, None), 
                           scrollable=True, vertical_scroll_only=True,expand_y=True ), 
                sg.VSeparator(), 
                sg.Column(image_viewer_column), ] 
             ]

    window = sg.Window("Auto Mode: Setup all initial values", layout)

    while True: # Run the Event Loop
        event, values = window.read()
        try:
            event_name = event.rstrip('0123456789')
            event_idx = int(event[len(event_name):])
            #print(event_name, event_idx)
        except:
            pass

        if event == "Cancel" or event == sg.WIN_CLOSED:
            selected_mode = 0 #default Panda pose
            selected_motion = 0 #default
            iterations = 1 #default number of iters.
            Exit_flag = 1 #EXit program
            Hide_prev = window["Hide_prev"].get()
            break

        elif event_name == "but":
            try:
                window["img"].update(source=dat[event_idx])
            except:
                #print("Error")
                pass

            window["but"+str(selected_mode)].update(text = "",  image_subsample = 5) #clear last one
            selected_mode = event_idx
            window["but"+str(selected_mode)].update(text = "SELECTED",  image_subsample = 5)
            window["-TOUT-"].update(value = "Description: " + desc[selected_mode])
            window['OK'].set_tooltip("You may Proceed!")
            window["OK"].update(disabled = False)               
            #print(selected_mode)
            
            if selected_mode == 4 or selected_mode == 5 or selected_mode == 8 or selected_mode == 9:
                window["manual_offset"].update(value = loaded_manual_offset, disabled = False, move_cursor_to = "end")
                window["manual_offset"].set_tooltip("Value loaded from saved settings. (in Meters)")
            else:
                window["manual_offset"].update(value = 0.0, disabled = True)
                window["manual_offset"].set_tooltip("Not required. Setting to 0")

        elif event == "OK":
            
            man_offset = window["manual_offset"].get()
            TGT_save = window["TGT_save"].get()
            cluster_idx = window["cluster_idx"].get()
            iterations = window["iterations"].get()
            Hide_prev = window["Hide_prev"].get()
            
            #print("The current selection includes:",selected_mode)
            break

        elif event == "Rad_Auto":
            selected_motion = 0 #default
        elif event == "Rad_X":  #curve along Y-axis and motion along X axis
            selected_motion = 2 
        elif event == "Rad_Y":  #curve along X-axis and motion along Y axis
            selected_motion = 3 

    window.close()

    return selected_mode, float(man_offset), int(cluster_idx), selected_motion, TGT_save, int(iterations), Hide_prev, Exit_flag

new_OP = Auto_gui(my_input[0],my_input[1],my_input[2])
print(new_OP)