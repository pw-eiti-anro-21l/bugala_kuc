import csv
import os
import numpy
# import mathutils
from ament_index_python.packages import get_package_share_directory
import transformations
from lab4_interfaces.srv import Interpolation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile
from math import floor
from time import sleep
from math import pi
# from visualization_msgs.msg import Marker, MarkerArray


import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from math import cos, sin, atan, atan2, sqrt, acos, asin, pi
from rclpy.qos import QoSProfile
import transformations
# import mathutils


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

dh = csv_reader('dh_table.csv')

class Jint(Node):

	def __init__(self):
		super().__init__('jint')
		self.srv = self.create_service(Interpolation, 'interpolation', self.interpolation_callback)
		qos_profile = QoSProfile(depth=10)
		# self.marker_pub = self.create_publisher(MarkerArray, '/marker', qos_profile)
		self.joint_pub = self.create_publisher(JointState, 'joint_interpolate', qos_profile)
		self.initial_position = [0, 0, 0]
		self.subscriber = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)


		self.base_height = 0.1
		self.length_1_2 = 0.5
		self.length_2_tool = 0.6



	def listener_callback(self, msg):
		for i in range(3):
			self.initial_position[i] = msg.position[i]

	def interpolation_callback(self, request, response):
		if request.time > 0:
			if request.joint_0_1_sv > pi:
				request.joint_0_1_sv = pi
			elif request.joint_0_1_sv < -pi:
				request.joint_0_1_sv = -pi

			if request.joint_1_2_sv > -0.3:
				request.joint_1_2_sv = -0.3
			elif request.joint_1_2_sv < -pi/2:
				request.joint_1_2_sv = pi/2

			if request.joint_2_3_sv > 1.22:
				request.joint_2_3_sv = 1.22
			elif request.joint_2_3_sv < -1.22:
				request.joint_2_3_sv = -1.22

			if request.method == "linear":
				self.linear_ip(request)
				response.output = f'Interpolation finished with linear method. Great job!'

			elif request.method == "trapezoid":
				self.trapezoid_ip(request)
				response.output = "Interpolation finished with trapezoid method. Great job!"
			
			else:
				response.output == "Method does not exist."
		else:
			response.output = "Error! Wrong time value."

		return response

	def linear_ip(self, request):
		sample_time = 0.01
		steps = floor(request.time/sample_time)
		joint_states = JointState()
		joint_states.name = ['joint_0_1', 'joint_1_2', 'joint_2_3']
		current_joint_states = self.initial_position

		# marker = Marker()
		# markers = MarkerArray()
		# marker.type = 2
		# marker.action = 0
		# marker.scale.x = 0.05
		# marker.scale.y = 0.05
		# marker.scale.z = 0.05
		# marker.color.a = 0.5
		# marker.color.r = 1.0
		# marker.color.g = 0.0
		# marker.color.b = 1.0
		# marker.header.frame_id = "/base"


		# for step in range(steps + 1):
		# 	joint_0_1_state = current_joint_states[0] + (request.joint_0_1_sv - current_joint_states[0])/steps*step
		# 	joint_1_2_state = current_joint_states[1] + (request.joint_1_2_sv - current_joint_states[1])/steps*step
		# 	joint_2_3_state = current_joint_states[2] + (request.joint_2_3_sv - current_joint_states[2])/steps*step
		# 	joint_states.position = [float(joint_0_1_state), float(joint_1_2_state), float(joint_2_3_state)]
			# 

		joints_v = self.find_joint_states([.6,.6,.2])
		for step in range(steps + 1):
			joint_0_1_state = current_joint_states[0] + (joints_v[0] - current_joint_states[0])/steps*step
			joint_1_2_state = current_joint_states[1] + (joints_v[1] - current_joint_states[1])/steps*step
			joint_2_3_state = current_joint_states[2] + (joints_v[2] - current_joint_states[2])/steps*step
			joint_states.position = [float(joint_0_1_state), float(joint_1_2_state), float(joint_2_3_state)]
	
			# 
			# 
			# 

			# 
			# 
			# 

			self.joint_pub.publish(joint_states)
			sleep(sample_time)

			# xyz_pose = find_tool([float(joint_0_1_state), float(joint_1_2_state), float(joint_2_3_state)]) 
			# marker.pose.position.x = xyz_pose[0]
			# marker.pose.position.y = xyz_pose[1]
			# marker.pose.position.z = xyz_pose[2]
			# # marker.pose.orientation = orientation_quaternion
			# markers.markers.append(marker)
			# id=0
			# for marker in markers.markers:
			# 	marker.id = id
			# 	id += 1
			# self.marker_pub.publish(markers)

		self.initial_position = [joint_0_1_state, joint_1_2_state, joint_2_3_state]

	def trapezoid_ip(self, request):
		sample_time = 0.01
		steps = floor(request.time/sample_time)
		joint_states = JointState()
		joint_states.name = ['joint_0_1', 'joint_1_2', 'joint_2_3']
		current_joint_states = self.initial_position
		v_max = [
		(request.joint_0_1_sv - current_joint_states[0]) / (request.time*.75),
		(request.joint_1_2_sv - current_joint_states[1]) / (request.time*.75),
		(request.joint_2_3_sv - current_joint_states[2]) / (request.time*.75)]
		v_curr = [0, 0, 0]

		# marker = Marker()
		# markers = MarkerArray()
		# marker.type = 2
		# marker.action = 0
		# marker.scale.x = 0.05
		# marker.scale.y = 0.05
		# marker.scale.z = 0.05
		# marker.color.a = 0.5
		# marker.color.r = 1.0
		# marker.color.g = 0.0
		# marker.color.b = 1.0
		# marker.header.frame_id = "/base"

		for step in range(steps + 1):
			for i in range(3):
				if step < 0.25*steps:
					v_curr[i] = v_max[i]*step/(.25*steps)
				elif step >= 0.25*steps and step <= 0.75*steps:
					v_curr[i] = v_max[i]
				elif step > 0.75 * steps:
					v_curr[i] = v_max[i] - v_max[i] * (step - .75*steps)/(.25*steps)
			for j in range(3):
				current_joint_states[j] = current_joint_states[j] + v_curr[j]*request.time/steps
				# joint_0_1_state = current_joint_states[]
			joint_states.position = [float(current_joint_states[0]), float(current_joint_states[1]), float(current_joint_states[2])]

			self.joint_pub.publish(joint_states)
			sleep(sample_time)

			self.joint_pub.publish(pose)
			# marker.pose.position.x = current_position[0]
			# marker.pose.position.y = current_position[1]
			# marker.pose.position.z = current_position[2]
			# marker.pose.orientation = orientation_quaternion
			# markers.markers.append(marker)
			# id=0
			# for marker in markers.markers:
			# 	marker.id = id
			# 	id += 1
			# self.marker_pub.publish(markers)
		
		self.initial_position = [current_joint_states[0], current_joint_states[1], current_joint_states[2]]

# def find_tool(joints):
# 	T = numpy.eye(4)
# 	# xyz_pose=[float(1),float(1),float(1)]
# 	for row in range(len(dh)-1):
# 		# print(rows)
# 		rot_theta = transformations.rotation_matrix(float(dh[row][4]+joints[row]), (0,0,1))
# 		rot_alpha = transformations.rotation_matrix(0.0, (1,0,0))
# 		trans_a = transformations.translation_matrix((dh[row+1][1],0,0))
# 		trans_d = transformations.translation_matrix((0,0,dh[row+1][2]))
# 		T_matrix = rot_alpha @ trans_a @ rot_theta @ trans_d
# 		T = T @ T_matrix
# 	xyz_pose = [T[0][3], T[1][3], T[2][3]]
# 	return xyz_pose
	def find_joint_states(self, point):
		x = point[0]
		y = point[1]
		z = point[2] - self.base_height
		a = self.length_1_2
		d = self.length_2_tool
		dist = sqrt(x*x + y*y + z*z)
		gamma = acos((a*a + d*d - dist*dist)/(2*a*d))
		joint_2_3 = -(pi - gamma)
		alpha = asin(d*sin(gamma)/dist)
		joint_1_2 = alpha + atan(z/sqrt(x*x + y*y))
		joint_0_1 = atan(y/x)
		return [joint_0_1, joint_1_2, joint_2_3]



def main(args=None):
	rclpy.init(args=args)

	jint = Jint()

	rclpy.spin(jint)

	rclpy.shutdown()


if __name__ == '__main__':
	main()