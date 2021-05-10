import sys
import rclpy
from rclpy.node import Node
from lab4_interfaces.srv import Interpolation

class Jint_client(Node):
	def __init__(self):
		super().__init__('jint_client')
		self.client = self.create_client(Interpolation, 'interpolation')
		while not self.client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')
		self.request = Interpolation.Request()

	def send_request(self):
		try:
			self.request.joint_0_1_sv = float(sys.argv[1])
			self.request.joint_1_2_sv = float(sys.argv[2])
			self.request.joint_2_3_sv = float(sys.argv[3])
			self.request.time = float(sys.argv[4])
			self.request.method = sys.argv[5]
			self.future = self.client.call_async(self.request)
			print([self.request.joint_0_1_sv, self.request.joint_1_2_sv, self.request.joint_2_3_sv, self.request.time, self.request.method])
		except:
			pass
	#     except ValueError:
	#         self.req.position_joint1 = -1.
	#         self.req.position_joint2 = -1.
	#         self.req.position_joint3 = -1.
	#         self.req.interpolation_time = -1.
	#         self.req.method = ""
	#         self.future = self.client.call_async(self.req)
	#     except IndexError:
	#         self.req.position_joint1 = -1.
	#         self.req.position_joint2 = -1.
	#         self.req.position_joint3 = -1.
	#         self.req.interpolation_time = -1.
	#         self.req.method = ""
	#         self.future = self.client.call_async(self.req)


def main(args=None):
	rclpy.init(args=args)

	jint_client = Jint_client()
	jint_client.send_request()

	while rclpy.ok():
		rclpy.spin_once(jint_client)
		if jint_client.future.done():
			try:
				response = jint_client.future.result()
			except Exception as e:
				jint_client.get_logger().info(
					'Service call failed %r' % (e,))
			else:
				jint_client.get_logger().info(response.output)
			break

	jint_client.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()