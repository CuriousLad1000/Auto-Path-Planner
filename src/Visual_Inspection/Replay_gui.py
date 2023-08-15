import PySimpleGUI as sg
import sys
from ast import literal_eval

my_input=sys.stdin.read()
my_input = literal_eval(my_input)


def Replay_gui(file_path, X_offset, Y_offset, Z_offset, TGT_save):
    font_small = ("gothic", 12)
    font_heading = ("gothic", 14)
    OK_flag = 0
    
    
    input_layout = [
        [sg.Text("Select Coordinate file:",font=font_heading)], 
        [sg.Input(default_text = file_path, key='file_path', visible=True, enable_events=True,readonly=True,), 
         sg.FileBrowse(file_types = (("Coordinate file","*.npy"),))],
        
        [sg.Text("The Offsets are added with respect to the World Coordinate system (wrt Robot's base)", 
                  auto_size_text = True, size=(50, 2), font=font_small, expand_y=True)],
        [sg.Text("")],
        [sg.Text("Add offset to X:", size=(25, 1), font=font_small, expand_y=True), 
         sg.Input(default_text = X_offset, size=(10, 1),key = "X_offset",tooltip = "(in Meters)" )],

        [sg.Text("Add offset to Y:", size=(25, 1), font=font_small, expand_y=True), 
         sg.Input(default_text = Y_offset, size=(10, 1),key = "Y_offset",tooltip = "(in Meters)" )],

        [sg.Text("Add offset to distance between camera and object:", size=(25, 2), font=font_small, expand_y=True), 
         sg.Input(default_text = Z_offset, size=(10, 1),key = "Z_offset",
              tooltip = "This will  OFFSET the distance between the camera and the object (in Meters)" )],
        
        [sg.Text("Save Targets?", size=(25, 1), auto_size_text = True , font=font_small, expand_y=True)
                    , sg.Checkbox("", default = TGT_save, key ="TGT_save",pad=((0, 0),(0,5))
                  , tooltip = "Save Generated Targets for later use.")]
                 ]
    

    buttons_layout = [
        
        [sg.Button(button_text = "Cancel",enable_events = True, tooltip ='Cancel', 
                                     size = (10, 2),key = 'Cancel', image_source=None,
                                     pad = ((0, 0), (0, 0)), 
                                     image_size = (None, None), image_subsample = None, ) ,
         
         sg.Button(button_text = "Set Initial Pose",enable_events = True, disabled = False, tooltip ='Set initial pose of Robot',
                                    size = (10, 2),key = 'init_pose', image_source=None,
                                     pad = ((20, 0), (0, 0)),
                                     image_size = (None, None), image_subsample = None ),
         
         sg.Button(button_text = "Preview",enable_events = True, disabled = True, tooltip ='Preview in Rviz',
                                     size = (10, 2),key = 'Preview', image_source=None,
                                     pad = ((20, 0), (0, 0)),
                                     image_size = (None, None), image_subsample = None ),
         
         sg.Button(button_text = "OK",enable_events = True, disabled = True, tooltip ='Select Coordinate file first!',
                                     size = (10, 2),key = 'OK', image_source=None,
                                     pad = ((20, 0), (0, 0)),
                                     image_size = (None, None), image_subsample = None )]
                 ]
    

    layout = [
                [input_layout], 
                [sg.HSeparator(pad=(0,10))],
                [buttons_layout] 
        
             ]

    window = sg.Window("Replay Mode: Coordinate Selector.", layout)
    window.timer_start(200, key='timer', repeating=False) #used to refresh events
    
    event, values = window.read()
    file_path = window["file_path"].get()    
    if file_path.find('.npy') > -1:
        window["Preview"].update(disabled = False)
        window["OK"].update(disabled = False)

    
    while True: # Run the Event Loop
        
        event, values = window.read()
        #print( event, values)
        try:
            event_name = event.rstrip('0123456789')
            event_idx = int(event[len(event_name):])
            #print(event_name, event_idx)
        except:
            pass

        
        if event == "Cancel" or event == sg.WIN_CLOSED:
            OK_flag = -1 #exit program
            
            break
            
        elif event == "file_path":
            window["file_path"].update(move_cursor_to = "end")
            file_path = window["file_path"].get()    
            if file_path.find('.npy') > -1:
                window["Preview"].update(disabled = False)
                window["OK"].update(disabled = False)
        
        elif event == "init_pose":
            
            OK_flag = 2 #set initial pose
            
            file_path = window["file_path"].get()
            X_offset = window["X_offset"].get()
            Y_offset = window["Y_offset"].get()
            Z_offset = window["Z_offset"].get()
            TGT_save = window["TGT_save"].get()
            
            break
        
        elif event == "Preview":
            OK_flag = 0 #preview flag

            file_path = window["file_path"].get()
            X_offset = window["X_offset"].get()
            Y_offset = window["Y_offset"].get()
            Z_offset = window["Z_offset"].get()
            TGT_save = window["TGT_save"].get()
            
            break
        elif event == "OK":
            OK_flag = 1 #preview flag
            
            file_path = window["file_path"].get()
            X_offset = window["X_offset"].get()
            Y_offset = window["Y_offset"].get()
            Z_offset = window["Z_offset"].get()
            TGT_save = window["TGT_save"].get()
            break

    window.close()

    return file_path, float(X_offset), float(Y_offset), float(Z_offset), TGT_save, int(OK_flag)

new_OP = Replay_gui(my_input[0],my_input[1],my_input[2],my_input[3],my_input[4])
print(new_OP)