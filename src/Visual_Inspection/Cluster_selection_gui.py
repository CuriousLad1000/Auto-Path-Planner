import PySimpleGUI as sg
import sys
from ast import literal_eval

my_input=sys.stdin.read()
my_input = literal_eval(my_input)

def Cluster_selection_gui(dat):

    selected_PC = [] #will hold index values of selected point clouds.

    file_list_column = [[sg.Text("Select all valid Clusters"),]]
    for i in range (len(dat)):
        #print(i)
        file_list_column.append( [sg.Button(button_text = "",enable_events = True, 
                                     size = (16, 4),key = 'but'+str(i), image_source=dat[i], 
                                     image_size = (None, None), image_subsample = 5, )] )

    buttons_layout = [
        [sg.Text("Buttons",size=(40, 1))],
        [ sg.Button(button_text = "Cancel",enable_events = True, 
                                     size = (8, 2),key = 'Cancel', image_source=None, 
                                     image_size = (None, None), image_subsample = None, ) ,
         sg.Button(button_text = "OK",enable_events = True, 
                                     size = (8, 2),key = 'OK', image_source=None, 
                                     image_size = (None, None), image_subsample = None, ) ] #to put buttons in single row
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

    window = sg.Window("Image Viewer", layout)

    while True: # Run the Event Loop
        event, values = window.read()
        try:
            event_name = event.rstrip('0123456789')
            event_idx = int(event[len(event_name):])
            #print(event_name, event_idx)
        except:
            pass

        if event == "Cancel" or event == sg.WIN_CLOSED:
            selected_PC = []
            break

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
                #print(selected_PC)
            else:
                selected_PC.append(event_idx)
                window["but"+str(event_idx)].update(text = "SELECTED",  image_subsample = 5)
                selected_PC = sorted(selected_PC)
                #print(selected_PC)

        elif event == "OK":
            #print("The current selection includes:",selected_PC)
            break

    window.close()

    return selected_PC

new_one = Cluster_selection_gui(my_input)
print(new_one)