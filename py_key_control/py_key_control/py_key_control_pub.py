import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType
from geometry_msgs.msg import Twist
import curtsies
from curtsies import Input

class ControlPublisher(Node):

	def __init__(self):
		super().__init__('control_publisher')
		self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
		timer_period = 0.1
		self.timer = self.create_timer(timer_period, self.key_control)
		self.declare_parameter('move_forward_key', 'i')
		self.declare_parameter('move_back_key', 'k')
		self.declare_parameter('turn_left_key', 'j')
		self.declare_parameter('turn_right_key', 'l')

	def key_control(self):
		twist = Twist()
		forward, back, left, right = self.get_params()
		with Input(keynames = 'curtsies') as input_generator:
			# while True:
			key = input_generator.send(0.1)
			if key == forward:
				twist.linear.x = 2.0
				twist.angular.z = 0.0
			elif key == back:
				twist.linear.x = -2.0
				twist.angular.z = 0.0
			elif key == left:
				twist.angular.z = 2.0
				twist.linear.x = 0.0
			elif key == right:
				twist.angular.z = -2.0
				twist.linear.x = 0.0
			elif key is None:
				twist.linear.x = 0.0
				twist.angular.z = 0.0

			self.publisher_.publish(twist)

	def get_params(self):
		forward_key = self.get_parameter('move_forward_key').get_parameter_value().string_value
		back_key = self.get_parameter('move_back_key').get_parameter_value().string_value
		left_key = self.get_parameter('turn_left_key').get_parameter_value().string_value
		right_key = self.get_parameter('turn_right_key').get_parameter_value().string_value
		return forward_key, back_key, left_key, right_key

def main(args = None):
	
	rclpy.init(args=args)

	control_publisher = ControlPublisher()

	print("ready")

	rclpy.spin(control_publisher)

	control_publisher.destroy_node()

	rclpy.shutdown()



if __name__ == "__main__":

	main()

