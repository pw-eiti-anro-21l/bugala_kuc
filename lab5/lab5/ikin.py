import rclpy
from rclpy.node import Node 
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from math import cos, sin, atan, atan2, sqrt, acos, asin, pi
from rclpy.qos import QoSProfile
import json

def get_params(part, filename):
	path = get_package_share_directory('lab5') + "/" + filename
	with open(path, "r") as file:
		read_file = json.load(file)
	part_params = read_file[part]
	return part_params

class Ikin(Node):

	def __init__(self):
		super().__init__('ikin')
		qos_profile = QoSProfile(depth=10)
		self.pose_sub = self.create_subscription(PoseStamped, '/ikin_pose', self.listener_callback, qos_profile)
		self.joint_pub = self.create_publisher(JointState, '/joint_interpolate', qos_profile)
		self.base_height = float(get_params('link_0', 'links_xyz.json')['length'])
		self.length_1_2 = float(get_params('link_2', 'links_xyz.json')['length'])
		self.length_2_tool = float(get_params('link_3', 'links_xyz.json')['length']) + float(get_params('link_4', 'links_xyz.json')['length'])

	def listener_callback(self, msg):
		self.find_joint_states(msg)

	def find_joint_states(self, pose):
		joint_states = JointState()
		joint_states.name = ['joint_0_1', 'joint_1_2', 'joint_2_3']
		x = pose.pose.position.x
		y = pose.pose.position.y
		z = pose.pose.position.z - self.base_height
		a = self.length_1_2
		d = self.length_2_tool
		dist = sqrt(x*x + y*y + z*z)
		if (dist<1.1):
			gamma = acos((a*a + d*d - dist*dist)/(2*a*d))
		else: 
			gamma = pi
		joint_2_3 = pi - gamma
		alpha = asin(d*sin(gamma)/dist)
		joint_1_2 = -(alpha + atan2(z,sqrt(x*x + y*y)))
		joint_0_1 = atan2(y,x)
		
		if (abs(joint_0_1)>3.14):
			self.get_logger().info("Error! Joint base->1 out of range.")
		elif (abs(joint_1_2+0.935)>0.635):
			self.get_logger().info("Error! Joint 1->2 out of range.")
		elif (abs(joint_2_3)>2):
			self.get_logger().info("Error! Joint 2->3 out of range.")
		else:
			joint_states.position = [float(joint_0_1), float(joint_1_2), float(joint_2_3)]
			self.joint_pub.publish(joint_states)




def main(args=None):
	rclpy.init(args=args)
	ikin = Ikin()
	rclpy.spin(ikin)
	rclpy.shutdown()

if __name__ == '__main__':
	main()

