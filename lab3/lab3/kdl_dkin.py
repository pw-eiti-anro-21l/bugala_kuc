import os
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
from PyKDL import *
import transformations
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

def find_rpy():
	dh = csv_reader('dh_table.csv')
	rows = len(dh)
	columns = len(dh[0])
	rpy_table = []
	xyz_table = []
	for row in range(0, rows):
		rot_theta = transformations.rotation_matrix(dh[row][4], (0,0,1))
		rot_alpha = transformations.rotation_matrix(dh[row][3], (1,0,0))
		trans_a = transformations.translation_matrix((dh[row][1],0,0))
		trans_d = transformations.translation_matrix((0,0,dh[row][2]))
		h_matrix = trans_a @ rot_alpha @ trans_d @ rot_theta
		rpy_table.append(transformations.euler_from_matrix(h_matrix))
		xyz_table.append(transformations.translation_from_matrix(h_matrix))
	return rpy_table, xyz_table

class Kdl(Node):

	def __init__(self):
		super().__init__('Kdl_dkin')

		self.subscription = self.create_subscription(
			JointState,
			'joint_states',
			self.listener_callback,
			10)
		self.subscription

	def listener_callback(self, msg):
	
		dh = csv_reader('dh_table.csv')
		rpy, xyz = find_rpy()
		chain = Chain()


		if (abs(msg.position[0])>1.0):
			self.get_logger().info("Error! Joint base->1 out of range.")
			
		elif (abs(msg.position[1]+0.935)>0.635):
			self.get_logger().info("Error! Joint 1->2 out of range.")
			
		elif (abs(msg.position[2])>1.57):
			self.get_logger().info("Error! Joint 2->3 out of range.")
		else:	

			joint_0_1 = Joint(Joint.RotZ)
			frame1 = Frame().DH(dh[1][1],dh[1][3],dh[0][2],dh[0][4])
			segment1 = Segment(joint_0_1, frame1)
			chain.addSegment(segment1)

			joint_1_2 = Joint(Joint.RotZ)
			frame2 = Frame().DH(dh[2][1],dh[2][3],dh[1][2],dh[1][4])
			segment2 = Segment(joint_1_2, frame2)
			chain.addSegment(segment2)

			joint_2_3 = Joint(Joint.RotZ)
			frame3 = Frame().DH(dh[3][1],dh[3][3],dh[2][2],dh[2][4])
			segment3 = Segment(joint_2_3, frame3)
			chain.addSegment(segment3)

			positions = JntArray(len(dh)-1)
			for i in range(len(dh)-1):
				positions[i] = msg.position[i]


			fk = ChainFkSolverPos_recursive(chain)
			lastFrame = Frame()
			fk.JntToCart(positions, lastFrame)

			quater = lastFrame.M.GetQuaternion()
			xyz_kdl = lastFrame.p

			pose_pub = self.create_publisher(PoseStamped, '/pose_stamped_kdl', QoSProfile(depth=10))

			pose = PoseStamped()
			pose.header.stamp = ROSClock().now().to_msg()
			pose.header.frame_id = "base"

			pose.pose.position.x = xyz_kdl[0]
			pose.pose.position.y = xyz_kdl[1]
			pose.pose.position.z = xyz_kdl[2]
			pose.pose.orientation = Quaternion(x=quater[0], y=quater[1], z=quater[2], w=quater[3])

			pose_pub.publish(pose)


def main(args=None):
	rclpy.init(args=args)

	kdl = Kdl()

	rclpy.spin(kdl)

	nonkdl.destroy_node()
	
	rclpy.shutdown()


if __name__ == '__main__':

	main()