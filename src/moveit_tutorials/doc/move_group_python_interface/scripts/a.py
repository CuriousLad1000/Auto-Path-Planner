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

from math import pi, tau, dist, fabs, cos
from moveit_commander.conversions import pose_to_list


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name) #we'll pass it on while calling functions

## Create a `DisplayTrajectory`_ ROS publisher which is used to display
## trajectories in Rviz:
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20,)


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

def go_to_coord_goal(move_group,xyz,RPY):

    quat = Rotation.from_euler('xyz', RPY, degrees=True).as_quat() #create a rotation object for converting to quaternion from rotation.
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]
    pose_goal.position.x = xyz[0]
    pose_goal.position.y = xyz[1]
    pose_goal.position.z = xyz[2]
    move_group.set_pose_target(pose_goal)
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    current_pose = move_group.get_current_pose().pose


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
    Roll_coeffs = [-1e-07, 6e-06, 0.0044, -1.3619, -88.761]
    Pitch_coeffs = [-5e-08, -6e-06, 0.0049, 0.1008, -45.236]
    yaw_coeffs = [-0.0053, 1.0159, -3.8925]


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


image_count = 94   #Number of images to cover (angles will be generated based on this number, first angle is set to 0 by default)
start_angle = 0.0  #Starting angle from where the coordinates are to be generated, subsequent angles are offsets over this one.

cx = 0.53  #This coordinate will align camera with center of roller towards x axis
cy = 0    #Center should be 0 this is center of roller not origin!
cz = 0.15  #center point of roller z axis

r = 0.1 + 0.25 #Radius of Roller (0.1) plus offset of 0.25
arc_len =  2 * r * pi/180  #angle to arc conversion  5 degree is the min. resolution
#arc_len = 0.01 #0.0268    #0.01     #in meters 10cm

xyz, RPY, angles = waypoints_generator(cx, cz, cy, r, arc_len, image_count,start_angle)


#Move to initial position
#go_to_coord_goal(move_group, [0.55, 0.0, 0.5], [-176, 1.0, 44.7]) #pass values to function to make robot move in cartesian space. 
go_to_coord_goal(move_group, [0.55, 0.0, 0.5], [-180, 0.0, 45]) #pass values to function to make robot move in cartesian space. 
go_to_coord_goal(move_group, [0.55, 0.0, 0.5], [0.0, 0.0, 0.0]) #pass values to function to make robot move in cartesian space. 
print("Moving to Initial state:")
print("")
time.sleep(2)


#for i in range (0,len(xyz)):

#    print("Coordinates at",angles[i],"degree: ", xyz[i], RPY[i])

#    go_to_coord_goal(move_group, xyz[i], RPY[i])

#    print("RPY In Degrees: ", np.degrees( move_group.get_current_rpy() ) )
#    print("")
#    #print(move_group.get_current_pose())
#    #input("============ Press `Enter` to execute a movement")
#    time.sleep(1)


#print("============Scan complete!")