import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from math import cos, sin, atan, atan2, sqrt
from rclpy.qos import QoSProfile
import transformations
import mathutils

class Ikin(Node):

	def __init__(self):
		super().__init__('ikin')
		qos_profile = QoSProfile(depth=10)
		self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
		self.pose_sub = self.create_subscription(PoseStamped, 'ikin_pose', self.listener_callback, qos_profile)
		