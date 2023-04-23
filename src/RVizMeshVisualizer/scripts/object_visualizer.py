#! /usr/bin/env python3
import os
import rospy
import rospkg
from visualization_msgs.msg import MarkerArray, Marker

rospy.init_node("object_visualizer")
rate = rospy.Rate(1)
rospy.loginfo('Initializing object visualizer')
rp = rospkg.RosPack()
visualizer_path = os.path.join(rp.get_path('object_visualizer'), 'meshes')
markerArray = MarkerArray()

publisher = rospy.Publisher('visualization_marker', MarkerArray, queue_size=1)

current_file_count = 0
while not rospy.is_shutdown():
    files = [file for file in os.listdir(visualizer_path) if file.endswith(('.dae', '.stl', '.mesh'))]  # find all meshes in the 'meshes' folder
    if len(files) != current_file_count:  # if the number of valid meshed in the 'meshes' folder has changed
        if len(files) < current_file_count:  # if some markers are removed from the 'meshes' folder, delete them in RViz
            marker = Marker()
            marker.header.frame_id = 'panda_link0'
            marker.action = marker.DELETEALL # send the DELETEALL marker to delete all marker in RViz
            markerArray.markers.append(marker)
            publisher.publish(markerArray)

        current_file_count = len(files)
        for marker_id, file in enumerate(files):
#            rospy.loginfo('Loading file: %s', file)
            marker = Marker()
            marker.id = marker_id
            marker.mesh_resource = 'package://object_visualizer/meshes/' + file
            marker.mesh_use_embedded_materials = True  # Need this to use textures for mesh
            marker.type = marker.MESH_RESOURCE
            marker.header.frame_id = "panda_link0"
            marker.scale.x = 0.001
            marker.scale.y = 0.001
            marker.scale.z = 0.001
            marker.pose.position.x = 0.70 - 0.10
            marker.pose.position.y = -0.08
            marker.pose.position.z = 0.15 - 0.10
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.7071068
            marker.pose.orientation.w = 0.7071068
            markerArray.markers.append(marker)

#    rospy.loginfo('Published %d objects. ', len(markerArray.markers))
    publisher.publish(markerArray)
    rate.sleep()
