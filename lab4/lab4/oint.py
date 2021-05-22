from lab4_interfaces.srv import OInterpolation as oInterpolation
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
# from visualization_msgs.msg import Marker, MarkerArray
from math import cos, sin, floor, pi
from rclpy.qos import QoSProfile
from time import sleep


class Oint(Node):

	def __init__(self):
		super().__init__('oint')
		self.srv = self.create_service(oInterpolation, 'oInterpolation', self.oInterpolation_callback)
		qos_profile = QoSProfile(depth=10)
		self.marker_pub = self.create_publisher(MarkerArray, '/marker', qos_profile)
		self.pose_pub = self.create_publisher(PoseStamped, 'oint_pose', qos_profile)
		self.initial_position = [0, 0, 0]
		self.initial_orientation = [0, 0, 0]

	def oInterpolation_callback(self, request, response):
		if request.time > 0:
			if request.method == "linear":
				self.linear_ip(request)
				response.output = "oInterpolation finished with linear method."
				self.initial_position = [0, 0, 0]
				self.initial_orientation = [0, 0, 0]

			elif request.method == "trapezoid":
				self.trapezoid_ip(request)
				response.output = "oInterpolation finished with trapezoid method."
				self.initial_position = [0, 0, 0]
				self.initial_orientation = [0, 0, 0]
			
			else:
				response.output == "This method does not exist."
		else:
			response.output = "Error! Value of time is invalid."

		return response

	def linear_ip(self, request):
		sample_time = 0.01
		steps = floor(request.time/sample_time)
		pose = PoseStamped()
		current_position = self.initial_position
		current_orientation = self.initial_orientation

		marker = Marker()
		markers = MarkerArray()
		marker.type = 2
		marker.action = 0
		marker.scale.x = 0.05
		marker.scale.y = 0.05
		marker.scale.z = 0.05
		marker.color.a = 0.5
		marker.color.r = 1.0
		marker.color.g = 0.0
		marker.color.b = 1.0
		marker.header.frame_id = "/base"
		
		for step in range(steps+1):
			pos_x = current_position[0] + (request.x_sv - current_position[0])/steps*step
			pos_y = current_position[1] + (request.y_sv - current_position[1])/steps*step
			pos_z = current_position[2] + (request.z_sv - current_position[2])/steps*step
			
			roll = current_orientation[0]  + (request.roll_sv - current_orientation[0])/steps*step
			pitch = current_orientation[1] + (request.pitch_sv - current_orientation[1])/steps*step
			yaw = current_orientation[2] + (request.yaw_sv - current_orientation[2])/steps*step
			orientation_quaternion = Quaternion(w=0.0, x=roll, y=pitch, z=yaw)

			pose.header.frame_id = "base"
			pose.pose.position.x = pos_x
			pose.pose.position.y = pos_y
			pose.pose.position.z = pos_z
			pose.pose.orientation = orientation_quaternion
			sleep(sample_time)
			
			self.pose_pub.publish(pose)
			marker.pose.position.x = pos_x
			marker.pose.position.y = pos_y
			marker.pose.position.z = pos_z
			marker.pose.orientation = orientation_quaternion
			markers.markers.append(marker)
			id=0
			for marker in markers.markers:
				marker.id = id
				id += 1
			self.marker_pub.publish(markers)

		self.initial_position = current_position
		self.initial_orientation = current_orientation

	def trapezoid_ip(self, request):
		sample_time = 0.01
		steps = floor(request.time/sample_time)
		pose = PoseStamped()
		current_position = self.initial_position
		current_orientation = self.initial_orientation


		v_max_pos = [
		(request.x_sv - current_position[0]) / (request.time*.75),
		(request.y_sv - current_position[1]) / (request.time*.75),
		(request.z_sv - current_position[2]) / (request.time*.75)]

		v_max_ort = [
		(request.roll_sv - current_orientation[0]) / (request.time*.75),
		(request.pitch_sv - current_orientation[1]) / (request.time*.75),
		(request.yaw_sv - current_orientation[2]) / (request.time*.75)]

		v_curr_pos = [0, 0, 0]
		v_curr_ort = [0, 0, 0]

		marker = Marker()
		markers = MarkerArray()
		marker.type = 2
		marker.action = 0
		marker.scale.x = 0.05
		marker.scale.y = 0.05
		marker.scale.z = 0.05
		marker.color.a = 0.5
		marker.color.r = 1.0
		marker.color.g = 0.0
		marker.color.b = 1.0
		marker.header.frame_id = "/base"

		for step in range(steps):
			for i in range(3):
				if step < 0.25*steps:
					v_curr_pos[i] = v_max_pos[i]*step/(.25*steps)
					v_curr_ort[i] = v_max_ort[i]*step/(.25*steps)
				elif step >= 0.25*steps and step <= 0.75*steps:
					v_curr_pos[i] = v_max_pos[i]
					v_curr_ort[i] = v_max_ort[i]
				elif step > 0.75 * steps:
					v_curr_pos[i] = v_max_pos[i] - v_max_pos[i] * (step - .75*steps)/(.25*steps)
					v_curr_ort[i] = v_max_ort[i] - v_max_ort[i] * (step - .75*steps)/(.25*steps)
			for i in range(3):
				current_position[i] += v_curr_pos[i]*request.time/steps
				current_orientation[i] += v_curr_ort[i]*request.time/steps
			orientation_quaternion = Quaternion(w=0.0, x=current_orientation[2], y=current_orientation[1], z=current_orientation[0])

			pose.header.frame_id = "base"
			pose.pose.position.x = current_position[0]
			pose.pose.position.y = current_position[1]
			pose.pose.position.z = current_position[2]
			pose.pose.orientation = orientation_quaternion
			sleep(sample_time)
			
			self.pose_pub.publish(pose)
			marker.pose.position.x = current_position[0]
			marker.pose.position.y = current_position[1]
			marker.pose.position.z = current_position[2]
			marker.pose.orientation = orientation_quaternion
			markers.markers.append(marker)
			id=0
			for marker in markers.markers:
				marker.id = id
				id += 1
			self.marker_pub.publish(markers)

		self.initial_position = current_position
		self.initial_orientation = current_orientation


def main(args=None):
	rclpy.init(args=args)

	oint = Oint()

	rclpy.spin(oint)

	rclpy.shutdown()


if __name__ == '__main__':
	main()