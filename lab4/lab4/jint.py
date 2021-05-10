from lab4_interfaces.srv import Interpolation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile
from math import floor
from time import sleep
from math import pi

class Jint(Node):

	def __init__(self):
		super().__init__('jint')
		self.srv = self.create_service(Interpolation, 'interpolation', self.interpolation_callback)
		qos_profile = QoSProfile(depth=10)
		self.joint_pub = self.create_publisher(JointState, 'joint_interpolate', qos_profile)
		self.initial_position = [0, 0, 0]
		self.subscriber = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)
		self.in_action = False

	def listener_callback(self, msg):
		# if not self.in_action:

		for i in range(3):
			self.initial_position[i] = msg.position[i]

	def interpolation_callback(self, request, response):
		# self.in_action = True
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
				response.output = "Linear interpolation completed."

			elif request.method == "trapezoid":
				self.trapezoid_ip(request)
				response.output == "Trapezoid interpolation completed."
			
			else:
				response.output == "Wrong method name."
		else:
			response.output = "Error! Wrong value of time."

		self.in_action = False

		return response

	def linear_ip(self, request):
		sample_time = 0.01
		steps = floor(request.time/sample_time)
		joint_states = JointState()
		joint_states.name = ['joint_0_1', 'joint_1_2', 'joint_2_3']
		current_joint_states = self.initial_position
		initial_joint_states = current_joint_states


		for step in range(steps):
			current_joint_states[0] += (request.joint_0_1_sv - initial_joint_states[0])/steps
			current_joint_states[1] += (request.joint_1_2_sv - initial_joint_states[1])/steps
			current_joint_states[2] += (request.joint_2_3_sv - initial_joint_states[2])/steps
			joint_states.position = [float(current_joint_states[0]), float(current_joint_states[1]), float(current_joint_states[2])]
			self.joint_pub.publish(joint_states)
			sleep(sample_time)

	def trapezoid_ip(self, request):
		sample_time = 0.01
		steps = floor(request.interpolation_time/sample_time)
		joint_states = JointState()
		joint_states.name = ['joint_0_1', 'joint_1_2', 'joint_2_3']
		current_joint_states = self.initial_position
		v_max = [
		(request.joint_0_1_sv - current_joint_states[0]) / (request.time*.75),
		(request.joint_1_2_sv - current_joint_states[1]) / (request.time*.75),
		(request.joint_2_3_sv - current_joint_states[2]) / (request.time*.75)]
		v_curr = [0, 0, 0]
		for step in range(steps):
			for i in range(3):
				if step < 0.25*steps:
					v_curr[i] = v_max[i]*step/(.25*steps)
				elif step >= 0.25*steps and step <= 0.75*steps:
					v_curr[i] = v_max[i]
				elif step > 0.75 * steps:
					v_curr[i] = v_max[i] - v_max[i] * (step - .75*steps)/(.25*steps)
			for i in range(3):
				current_joint_states[i] += v_curr[i]*request.time/steps
				joint_states.position[i] = float[current_joint_states[i]]

			self.joint_pub.publish(joint_states)
			sleep(sample_time)
		self.initial_position = [current_joint_states[0], current_joint_states[1], current_joint_states[2]]


def main(args=None):
	rclpy.init(args=args)

	jint = Jint()

	rclpy.spin(jint)

	rclpy.shutdown()


if __name__ == '__main__':
	main()