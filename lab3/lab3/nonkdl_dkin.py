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
from rclpy.clock import ROSClock
import time

def csv_reader(filename):
	dh = []
	with open(filename, 'r') as file:
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
	dh = csv_reader('~/dev_ws/src/bugala_kuc/lab3/lab3/dh_table.csv')
	T_matrix = []
	x_axis, z_axis = (1,0,0), (0,0,1)
	for joint in range (0,len(dh)-1):
		rot_alpha = mathutils.Matrix.Rotation(dh[joint][3], 4, 'X')
		rot_theta = mathutils.Matrix.Rotation(dh[joint][4], 4, 'Z')
		trans_a = mathutils.Matrix.Translation((dh[joint][1],0,0))
		trans_d = mathutils.Matrix.Translation((0,0,dh[joint][2]+position[joint]))
		T_joint = rot_alpha @ trans_a @  rot_theta @ trans_d
		if (len(T_matrix) != 0):
			T_matrix = T_matrix @ T_joint
		else:
			T_matrix = T_joint
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

		pose_pub = self.create_publisher(PoseStamped, '/pose_stamped_nonkdl', QoSProfile(depth=10))

		pose = PoseStamped()
		now = self.get_clock().now()
		pose.header.stamp = ROSClock().now().to_msg()
		pose.header.frame_id = "base"
		pose.pose.position.x = xyz[0]+0.1
		pose.pose.position.y = xyz[1]
		pose.pose.position.z = xyz[2]+0.1
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