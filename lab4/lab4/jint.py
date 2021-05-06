from lab4_interfaces.srv import Interplation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos impot QoSProfile


class Jint(Node):

    def __init__(self):
        super().__init__('jint')
        self.srv = self.create_service(Interpolation, 'interpolaion', self.interpolation_callback)
        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_interpolate', qos_profile)
        self.initial_position = [0, 0, 0]
        self.subscriber = self.create_subscription(JointState, 'joint_states', self.listener_callback, 10)
        self.in_action = False

    def listener_callback(self, msg):
    	if not in_action:
    		for i in range(3):
	    		self.initial_position[i] = msg.position[i]

	def interpolation_callback(self, request, response):
		self.in_action = True
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
				self.trapezoid_ip(request):
				response.output == "Trapezoid interpolation completed."
			
			else:
				response.output == "Wrong method name."
		else:
			response.output = "Error! Wrong value of time."

		self.in_action = False
		return response

	def linear_ip(self, request):
        sample_time = 0.01
        steps = floor(request.interpolation_time/sample_time)
        joint_states = JointState()
        joint_states.name = ['joint_base_1', 'joint_1_2', 'joint_2_3']
        start_joint_states = self.initial_joint_states

        for step in range(steps + 1):
            joint_1_state = start_joint_states[0] + (request.position_joint1 - start_joint_states[0])/steps*step
            joint_2_state = start_joint_states[1] + (request.position_joint2 - start_joint_states[1])/steps*step
            joint_3_state = start_joint_states[2] + (request.position_joint3 - start_joint_states[2])/steps*step
            joint_states.position = [float(joint_1_state), float(joint_2_state), float(joint_3_state)]
            self.joint_pub.publish(joint_states)
            sleep(sample_time)
        self.initial_joint_states = [joint_1_state, joint_2_state, joint_3_state]

def main(args=None):
    rclpy.init(args=args)

    jint = MinimalService()

    rclpy.spin(jint)

    rclpy.shutdown()


if __name__ == '__main__':
    main()