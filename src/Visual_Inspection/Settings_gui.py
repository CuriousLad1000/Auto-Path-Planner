import PySimpleGUI as sg
import sys
from ast import literal_eval

my_input=sys.stdin.read()
my_input = literal_eval(my_input)

def Settings_gui(data):
    selected_option = 0 #will hold which value was selected.

    samples = data[0] #1
    spacing = data[1] #0.01
    offset_y = data[2] #0.13
    offset_z = data[3] #0.0
    trim_base = data[4] #0.05
    manual_offset = data[5] #0.0
    Cluster_centered = data[6] #True

    if Cluster_centered == True:
        Cluster_idx = 0
    else:
        Cluster_idx = data[7] #200

    Cluster_discard = data[8] #0 #disabled else put number of pts as threshold
    eps = data[9] #0.05
    min_points = data[10] #10
    Cluster_trim = data[11] #0.01
    TGT_coord_Samples = data[12] #3
    TGT_final_trim = data[13] #0.8 #add max. euclidean distance away from robot's base (less than max reach of EEF) or z values less than...
    TGT_reverse = data[14] #True
    TGT_preview = data[15] #True
    z_offset = data[16] #0.3
    coord_skip = data[17] #3
    TGT_motion_delay = data[18] #0.1
    TGT_save = data[19] #True

    font_small = ("gothic", 12)
    font_heading = ("gothic", 14)

    text_layout = [[sg.Text("Point Cloud Settings", size=(60, 2), font=font_heading)],




                   [sg.Text("Number of pointcloud samples:", size=(60, 1), auto_size_text = True 
                            , font=font_small, expand_y=True), sg.Input(default_text = samples, size=(10, 1)
                            ,key = "PCD_Samples", pad = (0,0), focus=True
                            , tooltip = "number of samples to take before generating Point cloud" )],

                   [sg.Text("Set Point cloud Spacing", size=(60, 1)
                            , font=font_small), sg.Input(default_text = spacing
                    ,size=(10, 1),key = "PCD_spacing", pad = (0,0), tooltip = "PCD resolution in Meters" )],

                   [sg.Text("Y and Z offsets:", size=(60, 1), auto_size_text = True 
                            , font=font_small, expand_y=True), sg.Input(default_text = offset_y, size=(4, 1)
                                , key = "PCD_offset_y", pad = (0,0), tooltip = "Y offset in Meters" )
                   , sg.Input(default_text = offset_z, size=(4, 1), key = "PCD_offset_z", pad = (10,0)
                              , tooltip = "Z offset in Meters" )],

                   [sg.Text("Length from base to trim background from point cloud:", size=(60, 1)
                            , font=font_small, expand_y=True), sg.Input(default_text = trim_base, 
                                                             size=(10, 1),key = "trim_base", pad = (0,0)
                    , tooltip = "Trim any background points from Point cloud (in Meters)" )], 

                   [sg.Text("Offset distance between robot's base and Object", size=(60, 1)
                            , font=font_small, expand_y=True), sg.Input(default_text = manual_offset, 
                                                             size=(10, 1),key = "manual_offset", pad = (0,0) 
                    , tooltip = "useful if the object is not at same level as robot's base or in vertical scan (in Meters)")],



                   [sg.HSeparator(pad=(0,10))],

                   [sg.Text("Clustering settings ", size=(60, 2), auto_size_text = True 
                            , font=font_heading, expand_y=True)],

                   [sg.Text("Use Center of the object to generate Target point cloud points:"
                            , size=(60, 1), auto_size_text = True , font=font_small, expand_y=True)
                    , sg.Checkbox("", default = Cluster_centered, key ="Cluster_centered",enable_events=True
                                  ,pad=((0, 0),(0,10)))],

                   [sg.Text("Select index to generate target point cloud:"
                            , size=(60, 1), auto_size_text = True , font=font_small, expand_y=True)
                    , sg.Input(default_text = Cluster_idx, size=(10, 1),key = "Cluster_idx", pad = (0,0),disabled = Cluster_centered
                   , tooltip = "set index value to generate target point cloud, used only if centered tag is unchecked" )],


                   [sg.Text("Set threshold to discard Point clouds with less points. (0 will disable this feature)"
                            , size=(60, 2), auto_size_text = True , font=font_small, expand_y=True)
                    , sg.Input(default_text = Cluster_discard, size=(10, 1),key = "Cluster_discard", pad = (0,0)
                               , tooltip = "Minimum number of points required to be accepted as a cluster." )],

                   [sg.Text("Set Clustering parameters: Epsillon and Minimum number of points:"
                            , size=(60, 1), auto_size_text = True , font=font_small, expand_y=True)
                    ,sg.Input(default_text = eps, size=(4, 1)
                              , key = "Cluster_eps", pad = (0,0), tooltip = "Epsillon value" )
                   , sg.Input(default_text = min_points, size=(4, 1)
                              , key = "Cluster_min_pts", pad = (10,0), tooltip = "Minimum number of points" )],

                   [sg.Text("Trim ends of generated point cloud:"
                            , size=(60, 1), auto_size_text = True , font=font_small, expand_y=True)
                    , sg.Input(default_text = Cluster_trim, size=(10, 1),key = "Cluster_trim", pad = (0,0)
                               , tooltip = "Trim the edges of point cloud to remove outliers (in Meters)" )],


                   [sg.HSeparator(pad=(0,10))],

                   [sg.Text("Target Generation and Execution settings ", size=(60, 2), auto_size_text = True 
                            , font=font_heading, expand_y=True)],               

                   [sg.Text("Generated coordinate sets to reduce ghost target anomaly:", size=(60, 1)
                            , font=font_small, expand_y=True), sg.Input(default_text = TGT_coord_Samples
                    , size=(10, 1),key = "TGT_coord_Samples", pad = (0,0), tooltip = "reduces ghost targets, default: 3" )],

                   [sg.Text("Drop targets that are below certain threshold (use this to remove invalid targets that are out of reach or dangerous for the robot.)", 
                            size=(60, 2), font=font_small, expand_y=True)
                    , sg.Input(default_text = TGT_final_trim, size=(10, 1),key = "TGT_final_trim", pad = (0,0)
                              , tooltip = "remove all those targets that are in unsafe zone")],

                   [sg.Text("Reverse the order of the Targets:"
                            , size=(60, 1), auto_size_text = True , font=font_small, expand_y=True)
                    , sg.Checkbox("", default = TGT_reverse, key ="TGT_reverse",pad=((0, 0),(0,5))
                                  , tooltip = "Reverse the order ot Target execution")],

                   [sg.Text("Publish preview targets:"
                            , size=(60, 1), auto_size_text = True , font=font_small, expand_y=True)
                    , sg.Checkbox("", default = TGT_preview, key ="TGT_preview",pad=((0, 0),(0,5))
                                  , tooltip = "Preview Generated Targets in Rviz")],                

                   [sg.Text("Distance between Camera and Object:"
                            , size=(60, 1), auto_size_text = True , font=font_small, expand_y=True)
                    , sg.Input(default_text = z_offset, size=(10, 1),key = "TGT_Z_offset", pad = (0,0)
                   , tooltip = "This is Z distance from camera to object. Make sure to include EEF's distance too (in Meters)" )],


                   [sg.Text("Number of coordinate targets to skip while moving", size=(60, 1), font=font_small)
                    , sg.Input(default_text = coord_skip, size=(10, 1),key = "TGT_coord_skip", pad = (0,0)
                   , tooltip = "Used to scale down the total number of targets available:" )],

                   [sg.Text("Delay in robot's step motion", size=(60, 1), font=font_small)
                    , sg.Input(default_text = TGT_motion_delay,size=(10, 1),key = "TGT_motion_delay", pad = (0,0)
                   , tooltip = "Step delay between each target execution (in Sec)" )],

                   [sg.Text("Save Targets?", size=(60, 1), auto_size_text = True , font=font_small, expand_y=True)
                    , sg.Checkbox("", default = TGT_save, key ="TGT_save",pad=((0, 0),(0,5))
                  , tooltip = "Save Generated Targets for later use.")], 

                  ]




    buttons_layout = [[sg.Button(button_text = "Cancel",enable_events = True, tooltip ='Exit the program without saving changes.', 
                                     size = (10, 2),key = 'Exit', image_source=None,
                                     pad = ((60, 0), (0, 0)), 
                                     image_size = (None, None), image_subsample = None, ) ,
         sg.Button(button_text = "Save",enable_events = True, disabled = False, tooltip ="Let's Go!",
                                     size = (10, 2),key = 'Save', image_source=None,
                                     pad = ((20, 0), (0, 0)),
                                     image_size = (None, None), image_subsample = None )]]

    image_viewer_column = [
        [sg.Column(text_layout)],
        [sg.HSeparator(pad=(0,10))],
        [sg.Column(buttons_layout)],
    ]



    layout = [image_viewer_column]

    window = sg.Window("Settings", layout)


    while True: # Run the Event Loop
        event, values = window.read()
        #print(event, values)
        try:
            event_name = event.rstrip('0123456789')
            event_idx = int(event[len(event_name):])
            #print(event_name, event_idx)
        except:
            pass

        if event == "Exit" or event == sg.WIN_CLOSED:
            selected_option = -1 #default Exit the program
            break

        elif event == "Save":
            selected_option = 1  #Saving flag
            samples = window["PCD_Samples"].get()
            spacing = window["PCD_spacing"].get()
            offset_y = window["PCD_offset_y"].get()
            offset_z = window["PCD_offset_z"].get()
            trim_base = window["trim_base"].get()
            manual_offset = window["manual_offset"].get()
            Cluster_centered = window["Cluster_centered"].get()
            Cluster_idx = window["Cluster_idx"].get()
            Cluster_discard = window["Cluster_discard"].get() #disabled else put number of pts as threshold
            eps = window["Cluster_eps"].get()
            min_points=window["Cluster_min_pts"].get()
            Cluster_trim = window["Cluster_trim"].get()
            TGT_coord_Samples = window["TGT_coord_Samples"].get()
            TGT_final_trim = window["TGT_final_trim"].get() #add max. euclideaan...
            TGT_reverse = window["TGT_reverse"].get()
            TGT_preview = window["TGT_preview"].get()
            z_offset = window["TGT_Z_offset"].get()
            coord_skip = window["TGT_coord_skip"].get()
            TGT_motion_delay = window["TGT_motion_delay"].get()
            TGT_save = window["TGT_save"].get()


            break

        elif event == "Cluster_centered":

            #print(window["Cluster_centered"].get())
            if window["Cluster_centered"].get() == True:
                window["Cluster_idx"].update(disabled = True)
                window["Cluster_idx"].update(value = "0") #replace this to load previous value

            else:
                window["Cluster_idx"].update(disabled = False)
                window["Cluster_idx"].update(value = "0") 


    window.close()


    return selected_option, samples,spacing,offset_y,offset_z,trim_base,manual_offset,Cluster_centered,Cluster_idx,Cluster_discard,eps,min_points,Cluster_trim,TGT_coord_Samples,TGT_final_trim,TGT_reverse,TGT_preview,z_offset,coord_skip,TGT_motion_delay,TGT_save


new_OP = Settings_gui(my_input)
print(new_OP)