#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from moveit_commander.robot_trajectory import RobotTrajectory
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from moveit_commander.robot_trajectory import RobotTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface
import os

# Initialize MoveIt and rospy
rospy.init_node('add_leo_to_moveit', anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)

# Initialize Planning Scene Interface
scene = PlanningSceneInterface()
rospy.sleep(2)  # Give some time for scene to initialize

# Get the path to the Leo Rover URDF
leo_urdf_path = os.popen("rospack find leo_description").read().strip() + "/urdf/leo_rover.urdf.xacro"

# Create a new collision object for the Leo Rover
rover = moveit_commander.PlanningSceneInterface()
collision_object = moveit_commander.robot_trajectory.CollisionObject()

# Set the frame of reference (usually 'world' or base_link depending on your setup)
collision_object.header.frame_id = "world"  # You can use 'base_link' if you want the rover in relation to the arm.

# Set the name of the object (it should be unique)
collision_object.id = "leo_rover"

# Add the rover model (mesh or URDF) to the collision object
collision_object.meshes = [leo_urdf_path]  # Load the URDF model path as a mesh

# Create a pose for the rover and place it in the world
pose = PoseStamped()
pose.header.frame_id = "world"
pose.pose.position.x = 0.5  # Adjust position relative to the arm
pose.pose.position.y = 0.0
pose.pose.position.z = 0.1  # Place above the ground
pose.pose.orientation.w = 1.0  # No rotation

# Add the pose and mesh to the collision object
collision_object.primitive_poses = [pose.pose]
collision_object.operation = collision_object.ADD

# Add the collision object to the planning scene
scene.add_object(collision_object)
rospy.sleep(2)

print("Leo Rover added to the MoveIt planning scene!")

moveit_commander.roscpp_shutdown()
