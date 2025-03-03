import rospy
import moveit_commander
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped

rospy.init_node("add_leo_rover_collision")

scene = moveit_commander.PlanningSceneInterface()
rospy.sleep(1)  # Give time for the scene to initialize

# Define the Leo Rover collision object
leo_rover = CollisionObject()
leo_rover.id = "leo_rover"
leo_rover.header.frame_id = "world"  # Adjust if necessary

# Define the base shape
base = SolidPrimitive()
base.type = SolidPrimitive.BOX
base.dimensions = [0.425, 0.448, 0.305]  # Leo Rover dimensions in meters

# Define position
pose = PoseStamped()
pose.header.frame_id = "world"
pose.pose.position.x = 0.0
pose.pose.position.y = 0.0
pose.pose.position.z = 0.305 / 2  # Half of the height so it sits on the ground
pose.pose.orientation.w = 1.0  # No rotation

leo_rover.primitives.append(base)
leo_rover.primitive_poses.append(pose.pose)
leo_rover.operation = CollisionObject.ADD

# Add to the scene
scene.add_object(leo_rover)
rospy.sleep(1)

rospy.loginfo("Leo Rover collision object added.")
