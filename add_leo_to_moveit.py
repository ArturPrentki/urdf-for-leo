#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import subprocess
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import Mesh, MeshTriangle
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory
import os

# Initialize MoveIt
rospy.init_node('add_leo_rover_to_moveit', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
scene = moveit_commander.PlanningSceneInterface()
rospy.sleep(2)  # Allow some time for the scene to initialize

# Get Leo Rover URDF mesh path
leo_mesh_path = os.popen("rospack find leo_description").read().strip() + "/meshes/leo_body.stl"

# Define Leo Rover as a collision object
rover = CollisionObject()
rover.id = "leo_rover"
rover.header.frame_id = "world"  # Set the reference frame (or use "base_link")

# Load the rover mesh
rover_mesh = Mesh()
rover_mesh.triangles = []
rover_mesh.vertices = []

# Read mesh file
with open(leo_mesh_path, 'r') as mesh_file:
    for line in mesh_file:
        if line.startswith('v '):  # Vertex line
            _, x, y, z = line.split()
            rover_mesh.vertices.append([float(x), float(y), float(z)])
        elif line.startswith('f '):  # Face line
            _, v1, v2, v3 = line.split()
            rover_mesh.triangles.append([int(v1) - 1, int(v2) - 1, int(v3) - 1])

# Set the rover’s position
rover_pose = PoseStamped()
rover_pose.header.frame_id = "world"
rover_pose.pose.position.x = 0.5  # Adjust based on where you want the rover
rover_pose.pose.position.y = 0.0
rover_pose.pose.position.z = 0.1  # Keep it above ground
rover_pose.pose.orientation.w = 1.0

# Add the rover mesh to the collision object
rover.meshes.append(rover_mesh)
rover.mesh_poses.append(rover_pose.pose)
rover.operation = CollisionObject.ADD

# Publish to MoveIt’s planning scene
scene.add_object(rover)
rospy.sleep(2)

print("Leo Rover added to MoveIt planning scene!")

moveit_commander.roscpp_shutdown()
