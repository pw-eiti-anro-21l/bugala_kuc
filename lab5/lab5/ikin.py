import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from math import cos, sin, atan, atan2, sqrt, acos, asin, pi
from rclpy.qos import QoSProfile
import transformations
import mathutils

class Ikin(Node):

	def __init__(self):
		super().__init__('ikin')
		qos_profile = QoSProfile(depth=10)
		self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
		self.pose_sub = self.create_subscription(PoseStamped, 'ikin_pose', self.listener_callback, qos_profile)
		self.base_height = get_params('link_0', 'links_xyz.json')['length']
		self.length_1_2 = get_params('link_2', 'links_xyz.json')['length']
		self.length_2_tool = get_params('link_3', 'links_xyz.json')['length'] + get_params('link_4', 'links_xyz.json')['length']

	def get_params(part, filename):
	    with open(filename, "r") as file:
	        read_file = json.load(file)
	    part_params = read_file[part]
	    return part_params

	def listener_callback(self, msg):
		joint_states = JointState()
		joint_states.name = ['joint_0_1', 'joint_1_2', 'joint_2_3']

		new_joints = find_joint_states([1,1,1])
		new_joint_0_1 = new_joints[0]
		new_joint_1_2 = new_joints[1]
		new_joint_2_3 = new_joints[2]

		if (abs(new_joint_0_1)>3.14):
			self.get_logger().info("Error! Joint base->1 out of range.")
		elif (abs(new_joint_1_2+0.935)>0.635):
			self.get_logger().info("Error! Joint 1->2 out of range.")
		elif (abs(new_joint_2_3)>1.57):
			self.get_logger().info("Error! Joint 2->3 out of range.")
		else:
			joint_states.position = [float(new_joint_0_1), float(new_joint_1_2), float(new_joint_2_3)]
			self.joint_pub.publish(joint_states)

	def find_joint_states(self, point):
		x = point[0]
		y = point[1]
		z = point[2] - self.base_height
		a = self.length_1_2
		d = self.length_2_tool
		dist = sqrt(x*x + y*y + z*z)
		gamma = acos((a*a + d*d - dist*dist)/(2*a*d))
		joint_2_3 = pi - gamma
		alpha = asin(d*sin(gamma)/dist)
		joint_1_2 = alpha + atan2(z/sqrt(x*x + y*y))
		joint_0_1 = atan2(y/x)
		return [joint_0_1, joint_1_2, joint_2_3]




def main(args=None):
	rclpy.init(args=args)
	ikin = Ikin()
	rcply.spin(ikin)
	rclpy.shutdown()

if __name__ == '__main__':
	main()

