import PySimpleGUI as sg
import sys
from ast import literal_eval

my_input=sys.stdin.read()
my_input = literal_eval(my_input)


def Save_gui():
    font_small = ("gothic", 12)
    font_heading = ("gothic", 14)
    Save_mode = 0
    file_path='Select valid Coordinate file'
    
    
    radio_layout = [
        [sg.Radio(text = "Create a new file",default = True, enable_events = True, 
                  tooltip ="New file will be created based on mode of operation. If file exists, it will select different name",
                  key = 'Create_New_file', auto_size_text = True, group_id=0 , font=font_small )],

        [sg.Radio(text = "Overwrite previously saved file",default = False, enable_events = True, 
                  tooltip ='Warning... Coordinate file will be overwritten!',key = 'Overwrite_file',
                  auto_size_text = True, group_id=0, font=font_small )],

        [sg.Radio(text = "Append to a specific file",default = False, enable_events = True, 
                  tooltip ='Select a file to append new coordinates to...',key = 'Append_file',
                  auto_size_text = True, group_id=0, font=font_small )],

         ]

    input_layout = [        
        [sg.Text("Select Coordinate file:",font=font_heading, visible=False, key='file_txt')], 
        [sg.Input(default_text = file_path, key='file_path', visible=False, enable_events=True,readonly=True,), 
         sg.FileBrowse(file_types = (("Coordinate file","*.npy"),), visible=False, key='file_brw')]
                   ]

    buttons_layout = [
        
        [sg.Button(button_text = "Cancel",enable_events = True, tooltip ='Cancel', 
                                     size = (10, 2),key = 'Cancel', image_source=None,
                                     pad = ((45, 0), (0, 0)), 
                                     image_size = (None, None), image_subsample = None, ) ,

         sg.Button(button_text = "OK",enable_events = True, disabled = False, tooltip ='Save',
                                     size = (10, 2),key = 'OK', image_source=None,
                                     pad = ((20, 0), (0, 0)),
                                     image_size = (None, None), image_subsample = None )]
                 ]
    

    layout = [
                [radio_layout], 
                [input_layout],
                [sg.HSeparator(pad=(0,10))],
                [buttons_layout]
             ]

    window = sg.Window("Save Coordinates", layout)
    
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
            Save_mode = -1 #exit program
            
            break
            
        elif event == "file_path":
            window["file_path"].update(move_cursor_to = "end")
            file_path = window["file_path"].get()    
            if file_path.find('.npy') > -1:
                window["OK"].update(disabled = False)
        
        elif event == "OK":            
            file_path = window["file_path"].get()
            break

            
        elif event == "Create_New_file":
            Save_mode = 0 #default
        elif event == "Overwrite_file":
            Save_mode = 1 
        elif event == "Append_file":
            Save_mode = 2
            window["file_txt"].update(visible=True)
            window["file_path"].update(visible=True)
            window["file_brw"].update(visible=True)
            file_path = window["file_path"].get()    
            if file_path.find('.npy') > -1:
                window["OK"].update(disabled = False)
            else:
                window["OK"].update(disabled = True)

    window.close()

    return file_path, Save_mode

new_OP = Save_gui()
print(new_OP)