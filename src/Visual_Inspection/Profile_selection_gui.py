import PySimpleGUI as sg
import sys
from ast import literal_eval

import numpy as np
from PIL import Image
from io import BytesIO
import glob



my_input=sys.stdin.read()
my_input = literal_eval(my_input)


def Profile_selection_gui(X_profiles, Y_profiles, selected_Profiles=[]):
    Redundant=0
    Order=0
    dat=[]  #to hold cluster's images
    #selected_Profiles = [] #[[0,0]] - First one is the Option selection index goes from (0-5) 
                #Second one is the value selected based on the current option, and so, changes based on option
    selected_indexs = []
    used_indexs = []
    selected_key = -1
    OK_flag = 0  #Used to know the status of program Preview or OK or Cancel...
    
    mode_key = 0 #currently selected mode...
    mode_key_dict = {}  #Used to hold the currently selected profile mode to add, 
                        #will be used to keep track of pics to add to right side.
                        #and hold all the values for the selected profile before the final processing.
    
    font_small = ("gothic", 12)
    font_heading = ("gothic", 14)
    
    for f in sorted(glob.iglob("VI_appdata/Profile_Previews/*")):
        im = Image.open(f)
        output_buffer = BytesIO()
        im.save(output_buffer, format="PNG")
        data = output_buffer.getvalue()
        dat.append(data)
    

    file_list_column = [[sg.Text("Select the Object Profiles"),]]
    Selected_profiles_column = [[sg.Text("Selected Profiles in order"),]]
    
    
    for i in range (len(dat)): #Number of Options available
        #print(i)
        file_list_column.append( [sg.Button(button_text = "",enable_events = True, 
                                     size = (16, 4),key = 'but'+str(i), image_source=dat[i], 
                                     image_size = (None, None), image_subsample = 5, )] )

       
    buttons_layout = [
        [sg.Text("Control",size=(8, 2) )],
        
        [sg.Button(button_text = "Remove",enable_events = True, 
                                     size = (8, 2),key = 'Remove', image_source=None, 
                                     image_size = (None, None), image_subsample = None, ),    
         sg.Button(button_text = "Add",enable_events = True, 
                                     size = (8, 2),key = 'Add', image_source=None, 
                                     image_size = (None, None), image_subsample = None, ), ],
        

        
        [sg.Button(button_text = "Preview",enable_events = True, 
                                     size = (21, 2),key = 'Preview', image_source=None, disabled = True, 
                                     image_size = (None, None), image_subsample = None, )],
        [sg.Button(button_text = "Cancel",enable_events = True, 
                                     size = (8, 2),key = 'Cancel', image_source=None, 
                                     image_size = (None, None), image_subsample = None, ) ,
         sg.Button(button_text = "OK",enable_events = True, 
                                     size = (8, 2),key = 'OK', image_source=None, disabled = True,
                                     image_size = (None, None), image_subsample = None, ) ] #to put buttons in single row
                     ]
    
    tool_t_redundant = "Any profiles that appear twice will be filtered out."
    tool_t_order = "All unique profiles will be combined to mode 4 and mode 5 and removes redundancies automatically!"
    
    inputs_layout = [[sg.Text("Configurations", size=(30, 1), )],
                     
                     #[sg.Text("Enter the Number of X Profiles to generate:", size=(35, 1), auto_size_text = True 
                     #       , font=font_small, expand_y=True), sg.Input(default_text = int(np.floor(X_profiles/2)), size=(10, 1)
                     #       ,key = "txt_input", pad = (0,0), focus=True
                     #       , tooltip = "Number of object profiles to be generated in the direction of X axis. (Should be between 1 and "+str(X_profiles-1)+")" )],
                     
                     [sg.Text("Remove redundant profiles                             (Any profiles that appear twice will be filtered out):", 
                              size=(40, 2), auto_size_text = True, font=font_small, expand_y=True, tooltip = tool_t_redundant), 
                      sg.Checkbox("", default = Redundant, key ="Redundant",enable_events=True, 
                                  pad=((0, 0),(0,10)), tooltip = tool_t_redundant)],
                     
                     [sg.Text("Sort and Combine profiles into single X and Y profiles. (mode 4 and 5):", 
                              size=(40, 2), auto_size_text = True , font=font_small, expand_y=True, tooltip = tool_t_order)
                    , sg.Checkbox("", default = Order, key ="Order",enable_events=True, 
                                  pad=((0, 0),(0,10)), tooltip = tool_t_order)],
                     
                     [sg.Text("Enter the Number of X Profiles to generate:", size=(35, 2), auto_size_text = True 
                            , font=font_small, expand_y=True, key = 'txt', pad=((0, 0),(10,0)) ), ],

                     [sg.Slider(range = (1, X_profiles), default_value = int(np.floor(X_profiles/2)), orientation = 'h'
                               , size=(35, 20), key = 'slider', tooltip = "Number of object profiles to be generated in the direction of X axis. (Should be between 1 and "+str(X_profiles)+")"
                               , visible=True)],
                     
                   ]
    
    
    image_viewer_column = [
        [sg.Text("Profile Details")],
        #[sg.Text(size=(40, 1), key="-TOUT-")],
        [sg.Image(key="img",source=dat[0], subsample = 1,)], 
        [sg.HSeparator(pad=(0,10))],
        [sg.Column(inputs_layout, justification='left'), 
         sg.VSeparator(), sg.Column(buttons_layout, justification='right'),],
    
    ]


    layout = [ [ sg.Column(file_list_column,size = (None, None), 
                           scrollable=True, vertical_scroll_only=True,expand_y=True ), 
                 sg.VSeparator(), 
                 sg.Column(image_viewer_column), 
                 sg.VSeparator(), 
                 sg.Column(Selected_profiles_column,size = (None, None), 
                           scrollable=True, vertical_scroll_only=True,expand_y=True , key = 'selected'), ] 
             ]

    window = sg.Window("Profile Selector", layout)
    window.finalize()
    
    for loaded_profs in selected_Profiles:  #Load each profile...
        mode = loaded_profs[0]              #Extract profile number...
        loaded_prof = loaded_profs[1:]      #Extract Data...
        
        New_idx = np.random.randint(1000,9000, size=None)
        while (New_idx in selected_indexs) or (New_idx in used_indexs) : #Get a unique number
            New_idx = np.random.randint(1000,9000, size=None)  #selected_indexs[-1]+1
        if mode == 0:
            mode_key_dict[New_idx] = [mode, int(np.floor(X_profiles/2)-1)]
        elif mode == 1:
            mode_key_dict[New_idx] = [mode, int(np.floor(Y_profiles/2)-1)]
        elif mode == 2:
            ranges = loaded_prof
            ranges = np.insert(ranges, 0, mode, axis=None).tolist()
            mode_key_dict[New_idx] = ranges
        elif mode == 3:
            ranges = loaded_prof
            ranges = np.insert(ranges, 0, mode, axis=None).tolist()
            mode_key_dict[New_idx] = ranges
        elif mode == 4:
            X_profile_number = loaded_prof[0]+1 #int(values["slider"])
            mode_key_dict[New_idx] = [mode,X_profile_number-1]
        elif mode == 5:
            Y_profile_number = loaded_prof[0]+1 #int(values["slider"])                
            mode_key_dict[New_idx] = [mode,Y_profile_number-1]
            
        New_Selected_profile = [[sg.Button(button_text = "",enable_events = True, 
                                         size = (16, 4),key = 'L'+str(New_idx), image_source=dat[mode], 
                                         image_size = (None, None), image_subsample = 5, )]]

        window["OK"].update(disabled = False) #Enable OK button
        window["Preview"].update(disabled = False)
        selected_indexs.append(New_idx)

        window.extend_layout(window['selected'], New_Selected_profile)
        window.refresh()
        window['selected'].contents_changed()    # Update the content of `sg.Column` after window.refresh()   
    
    
    while True: # Run the Event Loop
        event, values = window.read()
        try:
            event_name = event.rstrip('0123456789')
            event_idx = int(event[len(event_name):])
            #print(event_name, event_idx)
        except:
            pass

        if event == "Cancel" or event == sg.WIN_CLOSED:
            selected_Profiles = []
            x_pros =[]
            y_pros=[] 
            OK_flag = -1 #exit program
            
            break

        elif event_name == "but":
            try:
                window["img"].update(source=dat[event_idx])
                window["but"+str(event_idx)].update(text = "SELECTED",  image_subsample = 5)
            except:
                #print("Error")
                
                pass
            
            if event_idx == 0:
                mode_key = 0
                window["txt"].update(value = 'Selecting the centered X profile...')
                window["slider"].update(value = int(np.floor(X_profiles/2)), range = (1, X_profiles), disabled = True)
                window.Element('slider').set_tooltip('Centered X profile selected!')
                
                #if ([0, int(np.floor(X_profiles/2))] not in selected_Profiles): #Option 0 and it's selected value
                #    selected_Profiles.append([0, int(np.floor(X_profiles/2))])
                    
                for i in [1,2,3,4,5]:
                    window["but"+str(i)].update(text = "", image_data=dat[i], image_subsample = 5)
                #print("Profile 0: One profile at center in X direction")
                
            elif event_idx == 1:
                mode_key = 1
                window["txt"].update(value = 'Selecting the centered Y profile...')
                window["slider"].update(value = int(np.floor(Y_profiles/2)), range = (1, Y_profiles), disabled = True)
                window.Element('slider').set_tooltip('Centered Y profile selected!')
                
                #if ([1, int(np.floor(Y_profiles/2))] not in selected_Profiles): #Option 1 and it's selected value
                #    selected_Profiles.append([1, int(np.floor(Y_profiles/2))])
                    
                for i in [0,2,3,4,5]:
                    window["but"+str(i)].update(text = "", image_data=dat[i], image_subsample = 5)
                #print("Profile 1: One profile at center in Y direction")
                
                
            elif event_idx == 2:
                mode_key = 2
                window["txt"].update(value = 'Enter the Number of X Profiles to generate:')
                window["slider"].update(value = 1, range = (1, X_profiles), disabled = False)
                window.Element('slider').set_tooltip("Number of equally spaced profiles generated in X direction. (Should be between 1 and "+str(X_profiles)+")")
                
                
                for i in [0,1,3,4,5]:
                    window["but"+str(i)].update(text = "", image_data=dat[i], image_subsample = 5)
                #print("Profile 2: Number of equally spaced profiles generated in X direction")
                
            elif event_idx == 3:
                mode_key = 3
                window["txt"].update(value = 'Enter the Number of Y Profiles to generate:')
                window["slider"].update(value = 1, range = (1, Y_profiles), disabled = False)
                window.Element('slider').set_tooltip("Number of equally spaced profiles generated in Y direction. (Should be between 1 and "+str(Y_profiles)+")")
                
                for i in [0,1,2,4,5]:
                    window["but"+str(i)].update(text = "", image_data=dat[i], image_subsample = 5)
                #print("Profile 3: Number of equally spaced profiles generated in Y direction")
                
            elif event_idx == 4:
                mode_key = 4
                window["txt"].update(value = 'Select the Profile number to generate targets in X direction:')
                window["slider"].update(value = 1, range = (1, X_profiles), disabled = False)
                window.Element('slider').set_tooltip("X profile number should be between 1 and "+str(X_profiles)+")")

                
                for i in [0,1,2,3,5]:
                    window["but"+str(i)].update(text = "", image_data=dat[i], image_subsample = 5)
                #print("Profile 4: In X direction (Selection within the range of Total number of X profiles available)")
                
            elif event_idx == 5:
                mode_key = 5
                window["txt"].update(value = 'Select the Profile number to generate targets in Y direction:')
                window["slider"].update(value = 1, range = (1, Y_profiles), disabled = False)
                window.Element('slider').set_tooltip("Y profile number should be between 1 and "+str(Y_profiles)+")")
                
                for i in [0,1,2,3,4]:
                    window["but"+str(i)].update(text = "", image_data=dat[i], image_subsample = 5)
                #print("Profile 5: In Y direction (Selection within the range of Total number of Y profiles available)")
                
                
        elif event_name == "L":
            try:

                #print("Selected Indexes On Click: ",selected_indexs)
                #print("Event Index On Click: ",event_idx)
                window["L"+str(event_idx)].update(text = "SELECTED",  image_subsample = 5)
                
                selected_key = event_idx  #Will be used to remove that particular entry
                
                curr_idxes = selected_indexs #Update all other buttons so that they are not SELECTED
                curr_idxes = [s for s in curr_idxes if s != event_idx]
                for i in curr_idxes:
                    window["L"+str(i)].update(text = "", image_data=dat[mode_key_dict[i][0]], image_subsample = 5)
            except:
                #print("Error")
                pass
            
        elif event == "Add":
            #print("Add Pressed")
            New_idx = np.random.randint(1000,9000, size=None)
            while (New_idx in selected_indexs) or (New_idx in used_indexs) : #Get a unique number
                New_idx = np.random.randint(1000,9000, size=None)  #selected_indexs[-1]+1
            #print("New Key Index numbers while adding:",New_idx)

            if mode_key == 0:
                #if ([mode_key, int(np.floor(X_profiles/2))] not in mode_key_dict.values()): #Option 0 and it's selected value
                mode_key_dict[New_idx] = [mode_key, int(np.floor(X_profiles/2)-1)]

            elif mode_key == 1:
                #if ([1, int(np.floor(Y_profiles/2))] not in mode_key_dict.values()): #Option 1 and it's selected value
                mode_key_dict[New_idx] = [mode_key, int(np.floor(Y_profiles/2)-1)]

            elif mode_key == 2:
                parts = int(values["slider"])
                ranges = np.round(np.linspace(0, X_profiles-1, num=parts)).astype('int')
                ranges = np.insert(ranges, 0, mode_key, axis=None).tolist()
                mode_key_dict[New_idx] = ranges

            elif mode_key == 3:
                parts = int(values["slider"])
                ranges = np.round(np.linspace(0, Y_profiles-1, num=parts)).astype('int')
                ranges = np.insert(ranges, 0, mode_key, axis=None).tolist()
                mode_key_dict[New_idx] = ranges
  
            elif mode_key == 4:
                X_profile_number = int(values["slider"])
                mode_key_dict[New_idx] = [mode_key,X_profile_number-1]

            elif mode_key == 5:
                Y_profile_number = int(values["slider"])                
                mode_key_dict[New_idx] = [mode_key,Y_profile_number-1]
                

            New_Selected_profile = [[sg.Button(button_text = "",enable_events = True, 
                                             size = (16, 4),key = 'L'+str(New_idx), image_source=dat[mode_key], 
                                             image_size = (None, None), image_subsample = 5, )]]
            
            window["OK"].update(disabled = False) #Enable OK button
            window["Preview"].update(disabled = False) #Enable OK button
            selected_indexs.append(New_idx)

            #print("Selected Indexes After Addition: ",selected_indexs) 
            #print("Selected Profile Modes: ",mode_key_dict.values())
            
            window.extend_layout(window['selected'], New_Selected_profile)
            window.refresh()
            window['selected'].contents_changed()    # Update the content of `sg.Column` after window.refresh()
            
        elif event == "Remove":

            if selected_key != -1:
                window['L'+str(selected_key)].update(visible=False)
                window['L'+str(selected_key)].Widget.master.pack_forget()

                selected_indexs.remove(selected_key)
                used_indexs.append(selected_key)
                mode_key_dict.pop(selected_key, None)
                
                #print("Selected Indexes After Removal: ",selected_indexs)
                #print("Selected Profile Modes: ",mode_key_dict.values())
                if len(selected_indexs) != 0:
                    selected_key = selected_indexs[-1] 
                    window["OK"].update(disabled = False)
                    window["Preview"].update(disabled = False)
                else:
                    selected_key = -1   #Nothing more left to select!
                    window["OK"].update(disabled = True)
                    window["Preview"].update(disabled = True)
            #else:
                #print("Select a Profile to Delete!")

            window['selected'].Widget.update()
            window['selected'].contents_changed()
            
        elif event == "Redundant":
            Redundant = int(values["Redundant"])
            #print("Redundant:",Redundant , "Order:", Order)
        elif event == "Order":
            Order = int(values["Order"])
            #print("Redundant:",Redundant , "Order:", Order)

        elif event == "Preview":
            OK_flag = 0 #preview flag
            break
    
        elif event == "OK":
            OK_flag = 1 #It's OK to proceed! flag
            break

    window.close()
    
    selected_Profiles = list(mode_key_dict.values())
    
    ### This block will extract unique profiles in x and y directions and 
    ### send them out to help functions easily extract specific profiles from the point cloud
    ### Remove dupes sort, combine to modes 4,5
    x_pros=[]
    y_pros=[]   
    for profiles in selected_Profiles:
        if profiles[0]%2 == 0: #even odd check
            x_pros += profiles[1:]  #combine all x modes (0,2,4) lists
        else:
            y_pros += profiles[1:]  #combine all y modes (1,3,5) lists

    x_pros = np.unique(x_pros).tolist()  #Remove dupes and sort and and convert to list add mode 4 tag 
    x_pros.insert(0,4)
    y_pros = np.unique(y_pros).tolist()  #Remove dupes and sort and and convert to list add mode 5 tag
    y_pros.insert(0,5)    
    
    
    if OK_flag == 1:  #Do this only if things are finalized and ready to proceed... and Not during Preview!!!
        if Order == 1 or (Redundant == 1 and Order == 1): #Remove dupes sort, combine to modes 4,5
            selected_Profiles = [x_pros,y_pros]  #Update final list to have both profiles

        elif Redundant == 1:  #Remove dupes Without sorting
            tmp = []
            for i in selected_Profiles:
                if i not in tmp:
                    tmp.append(i)
            selected_Profiles = tmp
        
    return selected_Profiles, x_pros, y_pros, int(OK_flag)

new_one = Profile_selection_gui(my_input[0], my_input[1], my_input[2])
print(new_one)