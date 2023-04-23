# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from scipy.spatial.transform import Rotation
import numpy as np
import time
import keyboard


try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list



def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

## First initialize `moveit_commander`_ and a `rospy`_ node:
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
## kinematic model and the robot's current joint states
robot = moveit_commander.RobotCommander()

## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
## for getting, setting, and updating the robot's internal understanding of the
## surrounding world:
scene = moveit_commander.PlanningSceneInterface()


## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
## to a planning group (group of joints).  In this tutorial the group is the primary
## arm joints in the Panda robot, so we set the group's name to "panda_arm".
## If you are using a different robot, change this value to the name of your robot
## arm planning group.
## This interface can be used to plan and execute motions:
group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name) #we'll pass it on while calling functions

## Create a `DisplayTrajectory`_ ROS publisher which is used to display
## trajectories in Rviz:
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
)

## Getting Basic Information
## ^^^^^^^^^^^^^^^^^^^^^^^^^
# We can get the name of the reference frame for this robot:
planning_frame = move_group.get_planning_frame()
print("============ Planning frame: %s" % planning_frame) #O/P: ============ Planning frame: panda_link0

# We can also print the name of the end-effector link for this group:
eef_link = move_group.get_end_effector_link()
print("============ End effector link: %s" % eef_link) #O/P: ============ End effector link: panda_link8

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
print("============ Available Planning Groups:", robot.get_group_names()) #O/P:  ============ Available Planning Groups: ['hand', 'panda_arm', 'panda_arm_hand']
print("")
# Sometimes for debugging it is useful to print the entire state of the
# robot:
#print("============ Printing robot state") #fetches the current joint state of the robot, all link,gripper positions in joint space
#print(robot.get_current_state())
#print("")

print("============ Printing robot current Joint values") #O/P: current joint values of the end-effector
print(move_group.get_current_joint_values())
print("In Degrees: ", np.degrees( move_group.get_current_joint_values() ) ) #O/P joint angles in degrees
print("")


print("============ Printing robot current pose") #O/P will be XYZ and xyz w  positions and orientations of the robot.
print(move_group.get_current_pose())
print("")

print("============ Printing robot current RPY") #O/P wil be orientation of end-effector in RPY (Radians)
print(move_group.get_current_rpy())
print("RPY In Degrees: ", np.degrees( move_group.get_current_rpy() ) )
print("")

#keyboard.unhook_all_hotkeys()
#Nxt = False
key = 'na'

def on_triggered_tab():
    print("Tab!")
    
def on_triggered_X():
    global key
    key = 'exit'
    #keyboard.unhook_all_hotkeys()
    #print("X!")
    
def on_triggered_R():
    global key
    key = "Reload"
    #print("R!")
    
def on_triggered_R_a():
    global key
    key = 'Roll_up'
    #print("Right arrow!")

def on_triggered_L_a():
    global key
    key = 'Roll_down'
    #print("Left arrow!")

def on_triggered_U_a():
    global key
    key = 'Pitch_up'
    #print("Up arrow!")
    
def on_triggered_D_a():
    global key
    key = 'Pitch_down'
    #print("Down arrow!")

def on_triggered_q():
    global key
    key = 'Yaw_up'
    #print("Q!")
    
def on_triggered_w():
    global key
    key = 'Yaw_down'
    #print("W!")

def on_triggered_enter():
    global key
    key = 'print_RPY'
    #print("Enter!")

def on_triggered_space():
    global key
    key = 'next_coord'
    #print("space!")

def on_triggered_p():
    global key
    key = 'previous'
    #print("P!")
    
def on_triggered_n():
    global key
    key = 'next'
    #print("N!")

def go_to_joint_state(move_group):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    
    ##move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
    ## thing we want to do is move it to a slightly better configuration.
    ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
    # We get the joint values from the group and change some of the values:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -tau / 8
    joint_goal[2] = 0
    joint_goal[3] = -tau / 4
    joint_goal[4] = 0
    joint_goal[5] = tau / 6  # 1/6 of a turn
    joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

def go_to_coord_goal(move_group,xyz,RPY):
    quat = Rotation.from_euler('xyz', RPY, degrees=True).as_quat() #create a rotation object for converting to quaternion from rotation.

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]
    pose_goal.position.x = xyz[0]
    pose_goal.position.y = xyz[1]
    pose_goal.position.z = xyz[2]

    move_group.set_pose_target(pose_goal)

    ## Now, we call the planner to compute the plan and execute it.
    # `go()` returns a boolean indicating whether the planning and execution was successful.
    success = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets().
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


def get_circle_coordinate(cx,cz,cy, r,theta): #generate x,z coordinates around a circle of radius r

    X = cx 
    Z = cz + ( r * np.sin( np.radians(180-theta) ) )
    Y = cy + ( r * np.cos( np.radians(180-theta) ) ) #subtract from 180 to make it clockwise coordinate generation, else, it'll start from 180 at 0 degree

    return X,Y,Z


def get_circle_RPY(theta):  #Predict RPY values based on the angle (in degrees)

    #Mode 1
#    Roll_coeffs = [-2.8252E-06, 0.000411511, -0.0246093, 1.654203, 98.10103]
#    Pitch_coeffs = [2.75083e-07, -4.56628e-05, 0.0107702, 0.0511628, -44.46201]
#    yaw_coeffs = [1.70203e-06, -0.000273718, 0.0210548, -1.24792, 83.988]

    #Mode 2
    Roll_coeffs = [-1e-07, 1e-05, 0.004, -1.3542, -88.897]
    Pitch_coeffs = [-5e-08, -8e-06, 0.0053, 0.0822, -45.198]
    yaw_coeffs = [-4e-09, 5e-06, -0.006, 1.0248, -3.4231]

    Roll_Order = len(Roll_coeffs)-1
    Pitch_Order = len(Pitch_coeffs)-1
    Yaw_Order = len(yaw_coeffs)-1
    Roll = 0
    Pitch = 0
    Yaw = 0
    
    for i in range (Roll_Order+1):
        Roll += Roll_coeffs[i] * theta**(Roll_Order - i)
    for i in range (Pitch_Order+1):
        Pitch += Pitch_coeffs[i] * theta**(Pitch_Order - i)    
    for i in range (Yaw_Order+1):
        Yaw += yaw_coeffs[i] * theta**(Yaw_Order - i)        
       
    return Roll, Pitch, Yaw  #in degrees


def waypoints_generator(cx, cz, cy, radius_plus_offset, arc_len, image_count, start_angle):

    waypoints_xyz = []
    waypoints_RPY = []
    angles =[]

    theta = (arc_len/radius_plus_offset) * 180/pi   #generate angle in degrees every theta degrees will be 1 arc_len

    for i in range (0, image_count+1):

        offset_angle = start_angle + theta*i
        X,Y,Z = get_circle_coordinate(cx, cz, cy, radius_plus_offset, offset_angle)
        R,P,Yaw = get_circle_RPY(offset_angle) #op in degrees
        waypoints_xyz.append([X,Y,Z])
        waypoints_RPY.append([ R,P,Yaw ])
        angles.append(np.round(offset_angle, decimals=1))

    return waypoints_xyz, waypoints_RPY, angles





print("")
print("----------------------------------------------------------")
print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
print("----------------------------------------------------------")
print("")


#cx = 1.57#0.67          #X coordinate of roller
#cz = 0.37          #Z coordinate of roller (in our case)
#r = 1.0+0.15     #Radius of roller plus offset distance between roller and gripper in meters
#arc_len = 0.01 #0.0268    #0.01     #in meters 10cm
image_count = 105   #Number of images to cover (angles will be generated based on this number, first angle is set to 0 by default)
start_angle = 0.0  #Starting angle from where the coordinates are to be generated, subsequent angles are offsets over this one.

cx = 0.53
cy = 0    #Center should be 0 this is center of roller not origin!
cz = 0.15  #centering the coordinate

r = 0.1 + 0.25 #Radius of Roller (0.1) plus offset of 0.25

#theta = 0  #in Degrees
#arc_len = 0.021806 #0.0268    #0.01     #in meters 10cm

arc_len =  2 * r * pi/180  #angle to arc conversion

angle_offset = 0.5

xyz, RPY, angles = waypoints_generator(cx, cz, cy, r, arc_len, image_count,start_angle)


#RR = [-90.0, -45.84168419,   2.12336684]
#RR = [-171.58466056,   -8.82905294,   44.3568652 ]
#RR = [-179.98090132,   -0.46626466,   43.05897968]

keyboard.add_hotkey('tab', on_triggered_tab)
keyboard.add_hotkey('x', on_triggered_X)

keyboard.add_hotkey('r', on_triggered_R)
keyboard.add_hotkey('right', on_triggered_R_a)

keyboard.add_hotkey('left', on_triggered_L_a)
keyboard.add_hotkey('x', on_triggered_X)

keyboard.add_hotkey('up', on_triggered_U_a)
keyboard.add_hotkey('down', on_triggered_D_a)

keyboard.add_hotkey('q', on_triggered_q)
keyboard.add_hotkey('w', on_triggered_w)

keyboard.add_hotkey('enter', on_triggered_enter)
keyboard.add_hotkey('space', on_triggered_space)

keyboard.add_hotkey('p', on_triggered_p)
keyboard.add_hotkey('n', on_triggered_n)
prev_XYZ = xyz[0]
prev_RPY = RPY[0]


for i in range (0,len(xyz)):
    if key == 'exit':
        break
    print("Coordinates at",angles[i],"degree: ", xyz[i], RPY[i]) #RPY[i] ) prev_RPY

    status = go_to_coord_goal(move_group, xyz[i], RPY[i]) #RPY[i]) prev_RPY  #pass values to function to make robot move in cartesian space. 
    print("Status:", status)
    print("")
    print("RPY In Degrees: ", np.degrees( move_group.get_current_rpy() ) )
    print("")
    #time.sleep(5)


    if i > 0:
        prev_XYZ = xyz[i-1]

    while True: #exit == False:

        if key == 'print_RPY':
            #print("current RPY values") #Enter
            print("Current RPY In Degrees: ", np.degrees( move_group.get_current_rpy() ) ) #Enter
            print("")
            key = 'na'


        elif key == 'Reload':
            #print("Reloading values") #R
            status = go_to_coord_goal(move_group, xyz[i], RPY[i])  #RPY[i]) prev_RPY #pass values to function to make robot move in cartesian space. 
            print("Status:", status)
            print("")
            print("Reloaded RPY In Degrees: ", np.degrees( move_group.get_current_rpy() ) )
            print("")
            key = 'na'

        elif key == 'next_coord':
            print("Moving to Next coordinate")  #Spacebar
            print("")
            key = 'na'
            prev_RPY = np.degrees( move_group.get_current_rpy() )  #store updated RPY values
            next_RPY = np.degrees( move_group.get_current_rpy() )
            break

        elif key == 'Roll_up':
            #print("Roll value up")   #Right arrow

            current_RPY = np.degrees( move_group.get_current_rpy() )
            new_RPY = [current_RPY[0]+angle_offset, current_RPY[1], current_RPY[2]]
            status = go_to_coord_goal(move_group, xyz[i], new_RPY)
            next_RPY = np.degrees( move_group.get_current_rpy() )
            print("Roll_RPY: ", np.degrees( move_group.get_current_rpy() ) )
            print("")
            key = 'na'

        elif key == 'Roll_down':
            #print("Roll value down") #Left arrow
            current_RPY = np.degrees( move_group.get_current_rpy() )
            new_RPY = [current_RPY[0]-angle_offset, current_RPY[1], current_RPY[2]]
            status = go_to_coord_goal(move_group, xyz[i], new_RPY) 
            next_RPY = np.degrees( move_group.get_current_rpy() )
            print("Roll_RPY: ", np.degrees( move_group.get_current_rpy() ) )
            print("")
            key = 'na'

        elif key == 'Pitch_up':
            #print("Pitch value up") #Up arrow
            current_RPY = np.degrees( move_group.get_current_rpy() )
            new_RPY = [current_RPY[0], current_RPY[1]+angle_offset, current_RPY[2]]
            status = go_to_coord_goal(move_group, xyz[i], new_RPY)
            next_RPY = np.degrees( move_group.get_current_rpy() )
            print("Pitch_RPY: ", np.degrees( move_group.get_current_rpy() ) )
            print("")
            key = 'na'

        elif key == 'Pitch_down':
            #print("Pitch value down")  #down arrow
            current_RPY = np.degrees( move_group.get_current_rpy() )
            new_RPY = [current_RPY[0], current_RPY[1]-angle_offset, current_RPY[2]]
            status = go_to_coord_goal(move_group, xyz[i], new_RPY) 
            next_RPY = np.degrees( move_group.get_current_rpy() )
            print("Pitch_RPY: ", np.degrees( move_group.get_current_rpy() ) )
            print("")
            key = 'na'

        elif key == 'Yaw_up':
            #print("Yaw value up") #q 
            current_RPY = np.degrees( move_group.get_current_rpy() )
            new_RPY = [current_RPY[0], current_RPY[1], current_RPY[2]+angle_offset]
            status = go_to_coord_goal(move_group, xyz[i], new_RPY)
            next_RPY = np.degrees( move_group.get_current_rpy() )
            print("Yaw_RPY: ", np.degrees( move_group.get_current_rpy() ) )
            print("")
            key = 'na'

        elif key == 'Yaw_down':
            #print("Yaw value down")  #w
            current_RPY = np.degrees( move_group.get_current_rpy() )
            new_RPY = [current_RPY[0], current_RPY[1], current_RPY[2]-angle_offset]
            status = go_to_coord_goal(move_group, xyz[i], new_RPY) 
            next_RPY = np.degrees( move_group.get_current_rpy() )
            print("Yaw_RPY: ", np.degrees( move_group.get_current_rpy() ) )
            print("")
            key = 'na'

        elif key == 'previous':
            status = go_to_coord_goal(move_group, prev_XYZ, prev_RPY)
            print("Previous RPY: ", np.degrees( move_group.get_current_rpy() ) )
            print("")
            key = 'na'

        elif key == 'next':

            status = go_to_coord_goal(move_group, xyz[i], next_RPY) 
            print("Next RPY: ", np.degrees( move_group.get_current_rpy() ) )
            print("")
            key = 'na'



        elif key == 'exit':
            print("Exiting")  #X key
            print("")
            #key = 'na'
            break

keyboard.unhook_all_hotkeys()

print("============ Python tutorial demo complete!")