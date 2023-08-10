import PySimpleGUI as sg
import sys
from ast import literal_eval

my_input=sys.stdin.read()
my_input = literal_eval(my_input)

def Cluster_selection_gui(dat):
    
    selected_PC = [] #will hold index values of selected point clouds.
    selected_motion = 0  #default auto

    file_list_column = [[sg.Text("Select all valid Clusters"),]]
    for i in range (len(dat)):
        #print(i)
        file_list_column.append( [sg.Button(button_text = "",enable_events = True, 
                                     size = (16, 4),key = 'but'+str(i), image_source=dat[i], 
                                     image_size = (None, None), image_subsample = 5, )] )

    buttons_layout = [
        [sg.Radio(text = "Auto select motion Path",default = True, enable_events = True, 
                  tooltip ='Program will decide motion Path',key = 'Rad_Auto',
                  auto_size_text = True, group_id=0, pad = ((5, 0), (0, 8)))],
        
        [sg.Radio(text = "Motion along X-axis",default = False, enable_events = True, 
                  tooltip ="Robot's motion along the camera's X axis",key = 'Rad_X',
                  auto_size_text = True, group_id=0 )],
        
        [sg.Radio(text = "Motion along Y-axis",default = False, enable_events = True, 
                  tooltip ="Robot's motion along the camera's Y axis",key = 'Rad_Y',
                  auto_size_text = True, group_id=0 ),          
         sg.Button(button_text = "Cancel",enable_events = True, tooltip ='First Cluster will be auto-selected', 
                                     size = (10, 2),key = 'Cancel', image_source=None,
                                     pad = ((60, 0), (0, 0)), 
                                     image_size = (None, None), image_subsample = None, ) ,
         sg.Button(button_text = "OK",enable_events = True, disabled = True, tooltip ='Select atleast one cluster',
                                     size = (10, 2),key = 'OK', image_source=None,
                                     pad = ((20, 0), (0, 0)),
                                     image_size = (None, None), image_subsample = None )]
                 ]
    
    
    
    image_viewer_column = [
        [sg.Text("Preview")],
        [sg.Text(size=(40, 1), key="-TOUT-")],
        [sg.Image(key="img",source=dat[0])], [sg.HSeparator(pad=(0,10))], [sg.Column(buttons_layout)]  ]


    layout = [ [ sg.Column(file_list_column,size = (None, None), 
                           scrollable=True, vertical_scroll_only=True,expand_y=True ), 
                sg.VSeparator(), 
                sg.Column(image_viewer_column), ] 
             ]

    window = sg.Window("Pointcloud Browser", layout)

    while True: # Run the Event Loop
        event, values = window.read()
        try:
            event_name = event.rstrip('0123456789')
            event_idx = int(event[len(event_name):])
            #print(event_name, event_idx)
        except:
            pass

        if event == "Cancel" or event == sg.WIN_CLOSED:
            selected_PC = [0] #default
            selected_motion = 0 #default
            break

        elif event == "Rad_Auto":
            selected_motion = 0 #default
        elif event == "Rad_X":  #curve along Y-axis and motion along X axis
            selected_motion = 2 
        elif event == "Rad_Y":  #curve along X-axis and motion along Y axis
            selected_motion = 3 

        elif event_name == "but":
            try:
                window["img"].update(source=dat[event_idx])
            except:
                print("Error")
                pass
            #if event_idx == 0:

            if (event_idx in selected_PC) == True:
                available_index = selected_PC.index(event_idx)
                del_item = selected_PC.pop(available_index)
                window["but"+str(event_idx)].update(text = "", image_data=dat[event_idx], image_subsample = 5)
                selected_PC = sorted(selected_PC)
                if len(selected_PC)==0:
                    window['OK'].set_tooltip("Select atleast one cluster")
                    window["OK"].update(disabled = True)
                #print(selected_PC)
            else:
                selected_PC.append(event_idx)
                window["but"+str(event_idx)].update(text = "SELECTED",  image_subsample = 5)
                window['OK'].set_tooltip("You may Proceed!")
                window["OK"].update(disabled = False)               
                selected_PC = sorted(selected_PC)
                #print(selected_PC)

        elif event == "OK":

            #print("The current selection includes:",selected_PC)
            break

    window.close()

    return selected_PC,selected_motion

new_one = Cluster_selection_gui(my_input)
print(new_one)