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

gripper_offset = 0.07  #only the white part
fingers_plus_gripper_offset = 0.115  #bot fingers included

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

def go_to_pose_goal(move_group):

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

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


def go_to_orient_goal(move_group,quat):

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = quat[0]
    pose_goal.orientation.y = quat[1]
    pose_goal.orientation.z = quat[2]
    pose_goal.orientation.w = quat[3]

    move_group.set_orientation_target(quat)

    success = move_group.go(wait=True)

    move_group.stop()    # Calling `stop()` ensures that there is no residual movement
    move_group.clear_pose_targets()

    current_pose = move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

def go_to_position_goal(move_group,gx,gy,gz):

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = gx
    pose_goal.position.y = gy
    pose_goal.position.z = gz
    move_group.set_position_target([gx,gy,gz])

    success = move_group.go(wait=True)

    move_group.stop()    # Calling `stop()` ensures that there is no residual movement
    move_group.clear_pose_targets()

    current_pose = move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)



def plan_cartesian_path(move_group, scale=1):

    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)  # waypoints to follow   # eef_step   # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

def display_trajectory(robot, display_trajectory_publisher, plan):

    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)    # Publish


def execute_plan(move_group, plan):

    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail


def wait_for_state_update(box_name, scene, box_is_known=False, box_is_attached=False, timeout=4):

    ## Ensuring Collision Updates Are Received
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
    ## or dies before actually publishing the scene update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed.
    ## To avoid waiting for scene updates like this at all, initialize the
    ## planning scene interface with  ``synchronous = True``.
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

def add_box(box_name, scene, timeout=4):

    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene between the fingers:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_hand"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.11  # above the panda_hand frame
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    #self.box_name = box_name
    
    return box_name, wait_for_state_update(box_name, scene, box_is_known=True, timeout=timeout)

def add_cylinder(cylinder_name, scene, timeout=4):

    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene between the fingers:
    cylinder_pose = geometry_msgs.msg.PoseStamped()
    cylinder_pose.header.frame_id = "panda_link0"
    
    cylinder_pose.pose.position.x = 0.57
    cylinder_pose.pose.position.y = 0.0
    cylinder_pose.pose.position.z = 0.37
    cylinder_pose.pose.orientation.x = 0.7068247749357853
    cylinder_pose.pose.orientation.y = 0.0
    cylinder_pose.pose.orientation.z = 0.0
    cylinder_pose.pose.orientation.w = 0.7073886750132324
    cylinder_name = "cylinder"
    scene.add_cylinder(cylinder_name, cylinder_pose, 0.314, 0.157)  #height, Radius

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    #self.box_name = box_name
    
    return cylinder_name, wait_for_state_update(cylinder_name, scene, box_is_known=True, timeout=timeout)

def attach_box(box_name, scene, robot, eef_link, group_names, timeout=4):

    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = "panda_hand"
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)

    # We wait for the planning scene to update.
    return wait_for_state_update(box_name, scene, box_is_attached=True, box_is_known=False, timeout=timeout)


def detach_box(box_name, scene, eef_link, timeout=4):

    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)

    # We wait for the planning scene to update.
    return wait_for_state_update(box_name, scene, box_is_known=True, box_is_attached=False, timeout=timeout)


def remove_box(box_name, scene, timeout=4):

    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return wait_for_state_update(box_name, scene, box_is_attached=False, box_is_known=False, timeout=timeout)

def obj_pose(object_name, scene): #Get the poses from the objects identified by the given object ids list.
    pos = scene.get_object_poses([object_name])   #This parameter expects a "list" of object IDs
    return pos

def get_circle_coordinate(cx,cz,cy, r,theta): #generate x,z coordinates around a circle of radius r

    X = cx + ( r * np.cos( np.radians(180-theta) ) ) #subtract from 180 to make it clockwise coordinate generation, else, it'll start from 180 at 0 degree
    Z = cz + ( r * np.sin( np.radians(180-theta) ) )
    Y = cy

    return X,Y,Z

def get_circle_RPY(theta):  #Generate RPY values based on the angle (in degrees)

#    Roll = 4*((10**-7)*theta**4) - 5*((10**-5)*theta**3) - 0.0026*(theta**2) + 1.2163*(theta) + 99.048
#    Pitch = -4*((10**-7)*theta**4) + 6*((10**-5)*theta**3) + 0.0014*(theta**2) + 0.1385*(theta) - 44.872
#    Yaw = 2*((10**-7)*theta**4) - 4*((10**-5)*theta**3) + 0.0078*(theta**2) - 0.9681*(theta) + 85.751

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

#input("============ Press `Enter` to execute a movement using a joint state goal ...")
#status = go_to_joint_state(move_group) #get out of singularity
#print("Status:", status)
#print("")



#input("============ Press `Enter` to add a cylinder to the planning scene ...")
#cylinder_name=""
#cylinder_name, status = add_cylinder(cylinder_name, scene)

#input("============ Press `Enter` to get object pose from the planning scene ...")
#p = obj_pose(cylinder_name, scene)
#p_center = p['cylinder'].position #fetch only the positions These positions are the coordinates of the center of the object.
#print(p)
#print("")

#print("Center coordinates of Cylinder: ",p_center) 
#print("")

#input("============ Press `Enter` to execute a movement using a pose goal ...")

#roll = 117.28508733 #0.0#45.0-90 #117.28508733
#pitch = -40.96505291 #70.0#45.0+90.0-20-20-20  #-40.96505291
#yaw = 72.50412602 #0.0#10.0-20-20 #72.50412602


#x = 0.3#-fingers_plus_gripper_offset
#y = 0.0
#z = 0.37

#rot = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True) #create a rotation object for converting to quaternion from rotation.
#quat = rot.as_quat() #Store quaternions in quat variable.



#status = go_to_coord_goal(move_group, x,y,z, quat) #pass values to function to make robot move in cartesian space. 
#print("Status:", status)
#print("")


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
#theta = 0  #in Degrees
#arc_len = 0.021806 #0.0268    #0.01     #in meters 10cm
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




#input("============ Press `Enter` to execute a movement using a Orientation goal ...")
#roll = 0.0
#pitch = 0.0
#yaw = 90.0
#status = go_to_orient_goal(move_group, quat)
#print("Status:", status)
#print("")

#input("============ Press `Enter` to execute a movement using a position goal ...")
#x = 0.87  #-fingers_plus_gripper_offset
#y = 0.0
#z = 0.37

#status = go_to_position_goal(move_group,x,y,z)
#print("Status:", status)
#print("")




print("============ Python tutorial demo complete!")