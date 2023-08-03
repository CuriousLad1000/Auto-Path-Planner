import PySimpleGUI as sg
import sys
from ast import literal_eval

my_input=sys.stdin.read()
my_input = literal_eval(my_input)

def Front_gui(data_in):
    selected_option = 0 #will hold which value was selected.

    radio_layout = [
        [sg.Radio(text = "Re-run the previously saved motion",default = True, enable_events = True, 
                  tooltip ="Robot's motion will depend on previously saved coordinates.",key = 'Replay',
                  auto_size_text = True, group_id=0 , font=("gothic", 12))],

        [sg.Radio(text = "Scan the Object (Manual mode)",default = False, enable_events = True, 
                  tooltip ='User will provide inputs at various places',key = 'Manual',
                  auto_size_text = True, group_id=0, font=("gothic", 12) )],

        [sg.Radio(text = "Scan the Object (Automatic mode)",default = False, enable_events = True, 
                  tooltip ='Program will iterate over manual-mode once it is setup.',key = 'Auto',
                  auto_size_text = True, group_id=0, font=("gothic", 12) )],
        [sg.Radio(text = "Change Default settings.",default = False, enable_events = True, 
                  tooltip ='Change the program presets and finetune default parameters.',key = 'Settings',
                  auto_size_text = True, group_id=0, font=("gothic", 12) )]
                 ]

    buttons_layout = [[sg.Button(button_text = "Exit",enable_events = True, tooltip ='Exit the program', 
                                     size = (10, 2),key = 'Exit', image_source=None,
                                     pad = ((60, 0), (0, 0)), 
                                     image_size = (None, None), image_subsample = None, ) ,
         sg.Button(button_text = "OK",enable_events = True, disabled = False, tooltip ="Let's Go!",
                                     size = (10, 2),key = 'OK', image_source=None,
                                     pad = ((20, 0), (0, 0)),
                                     image_size = (None, None), image_subsample = None )]]

    image_viewer_column = [
        [sg.Text("What would you like to do?", size=(40, 1), auto_size_text = True , font=("gothic", 14))],
        [sg.Column(radio_layout)],
        [sg.HSeparator(pad=(0,10))],
        [sg.Column(buttons_layout)],
    ]



    layout = [image_viewer_column]

    window = sg.Window("Cognitive Motion Planning", layout)


    while True: # Run the Event Loop
        event, values = window.read()
        try:
            event_name = event.rstrip('0123456789')
            event_idx = int(event[len(event_name):])
            print(event_name, event_idx)
        except:
            pass

        if event == "Exit" or event == sg.WIN_CLOSED:
            selected_option = -1 #default Exit the program
            break

        elif event == "Replay":
            selected_option = 0 #Replay coords
        elif event == "Manual":  #Manually select stuff
            selected_option = 1 
        elif event == "Auto":  #Auto select stuff
            selected_option = 2
        elif event == "Settings":  #Change settings
            selected_option = 3 
        elif event == "OK":

            #print("The current selection includes:",selected_option)
            break

    window.close()
    return selected_option

new_OP = Front_gui(my_input)
print(new_OP)