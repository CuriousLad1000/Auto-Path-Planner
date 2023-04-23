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


try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


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

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()


group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name) #we'll pass it on while calling functions

## Create a `DisplayTrajectory`_ ROS publisher which is used to display
## trajectories in Rviz:
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20,)

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

    X = cx + ( r * np.cos( np.radians(180-theta) ) ) #subtract from 180 to make it clockwise coordinate generation, else, it'll start from 180 at 0 degree
    Z = cz + ( r * np.sin( np.radians(180-theta) ) )
    Y = cy

    return X,Y,Z

def get_circle_RPY(theta):  #Generate RPY values based on the angle (in degrees)

    Roll = -2.8252*((10**-6)*theta**4) + 0.000411511*(theta**3) - 0.0246093*(theta**2) + 1.654203*(theta) + 98.10103
    Pitch = -2.75083*((10**-7)*theta**4) - 4.56628*((10**-5)*theta**3) + 0.0107702*(theta**2) + 0.0511628*(theta) - 44.46201
    Yaw = 1.70203*((10**-6)*theta**4) - 0.000273718*(theta**3) + 0.0210548*(theta**2) - 1.24792*(theta) + 83.988

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


minimum_eef_clearance = 0.363 # minimum required for EEF to function at current offsets
#cx = 1.57#0.67          #X coordinate of roller
#cz = 0.37          #Z coordinate of roller (in our case)
#r = 1.0+0.15     #Radius of roller plus offset distance between roller and gripper in meters
#arc_len = 0.01 #0.0268    #0.01     #in meters 10cm
image_count = 28   #Number of images to cover (angles will be generated based on this number, first angle is set to 0 by default)
start_angle = 0.0  #Starting angle from where the coordinates are to be generated, subsequent angles are offsets over this one.

cx = 0.80
cz = 0.37
cy = 0

r = 0.10+0.25#0.157+0.15

arc_len =  2.5 * r * pi/180  #angle to arc conversion



xyz, RPY, angles = waypoints_generator(cx, cz, cy, r, arc_len, image_count,start_angle)

#status = go_to_coord_goal(move_group, xyz[0], RPY[0]) #pass values to function to make robot move in cartesian space. 
#print("Status:", status)
#print("")

for i in range (0,len(xyz)):

    print("Coordinates at",angles[i],"degree: ", xyz[i], RPY[i] )

    status = go_to_coord_goal(move_group, xyz[i], RPY[i]) #pass values to function to make robot move in cartesian space. 
    print("Status:", status)
    print("")
    print("RPY In Degrees: ", np.degrees( move_group.get_current_rpy() ) )
    print("")
    #input("============ Press `Enter` to execute a movement")
    time.sleep(2)
    #print("")
    #print("Updated RPY In Degrees: ", np.degrees( move_group.get_current_rpy() ) )
    #print("")

#    print("============ Printing robot current Joint values at: ",angles[i],"degrees") #O/P: current joint values of the end-effector
#    print(move_group.get_current_joint_values())
#    print("In Degrees: ", np.degrees( move_group.get_current_joint_values() ) ) #O/P joint angles in degrees
#    print("")



print("============ Python tutorial demo complete!")