from math import sin, cos, pi
import os
import mathutils
import csv
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, TransformStamped
from ament_index_python.packages import get_package_share_directory
from rclpy.clock import ROSClock
import time

def csv_reader(filename):
	dh = []
	with open(os.path.join(get_package_share_directory('lab3'), filename), 'r') as file:
		read_file = csv.reader(file, delimiter = ';')
		for row in read_file:
			dh.append(row)
	rows = len(dh)
	columns = len(dh[0])
	for i in range(1, rows):
		for j in range(1, columns-1):
			dh[i][j] = float(dh[i][j])
	dh.pop(0)
	return dh

def solve(position):
	dh = csv_reader('dh_table.csv')
	T_matrix = []
	x_axis, z_axis = (1,0,0), (0,0,1)
	for joint in range (0,len(dh)):
		rot_alpha = mathutils.Matrix.Rotation(dh[joint][3], 4, 'X')
		rot_theta = mathutils.Matrix.Rotation(dh[joint][4], 4, 'Z')
		trans_a = mathutils.Matrix.Translation((dh[joint][1],0,0))
		trans_d = mathutils.Matrix.Translation((0,0,dh[joint][2]))
		if (joint < 3):
			rot_theta = mathutils.Matrix.Rotation(dh[joint][4]+position[joint], 4, 'Z')
		T_joint = rot_alpha @ trans_a @  rot_theta @ trans_d
		if (len(T_matrix) == 0):
			T_matrix = T_joint
		else:
			T_matrix = T_matrix @ T_joint
	return T_matrix

class NonKdl(Node):


	def __init__(self):
		super().__init__('NonKdl_dkin')

		self.subscription = self.create_subscription(
			JointState,
			'joint_states',
			self.listener_callback,
			10)
		self.subscription

	def listener_callback(self, msg):

		T = solve(msg.position)
		xyz = T.to_translation()
		rpy = T.to_euler()
		quater = rpy.to_quaternion()

		if (abs(msg.position[0])>1.0):
			self.get_logger().info("Error! Joint base->1 out of range.")
		elif (abs(msg.position[1]+0.935)>0.635):
			self.get_logger().info("Error! Joint 1->2 out of range.")
		elif (abs(msg.position[2])>1.57):
			self.get_logger().info("Error! Joint 2->3 out of range.")
		else:

			pose_pub = self.create_publisher(PoseStamped, '/pose_stamped_nonkdl', QoSProfile(depth=10))

			pose = PoseStamped()
			now = self.get_clock().now()
			pose.header.stamp = ROSClock().now().to_msg()
			pose.header.frame_id = "base"
			pose.pose.position.x = xyz[0]
			pose.pose.position.y = xyz[1]
			pose.pose.position.z = xyz[2]

			pose.pose.orientation = Quaternion(w=quater[0], x=quater[1], y=quater[2], z=quater[3])
			pose_pub.publish(pose)


def main(args=None):
	rclpy.init(args=args)

	nonkdl = NonKdl()

	rclpy.spin(nonkdl)

	nonkdl.destroy_node()
	
	rclpy.shutdown()


if __name__ == '__main__':

	main()