import sys
import rclpy
from rclpy.node import Node
from zadanie4_interface.srv import oInterpolation

class Oint_client(Node):
	def __init__(self):
		super().__init__('oint_client')
		self.client = self.create_client(ooInterpolation, 'ooInterpolation')
		while not self.client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')
		self.request = oInterpolation.Request()

	def send_request(self):
		try:
			self.request.joint_0_1_sv = float(sys.argv[1])
			self.request.joint_1_2_sv = float(sys.argv[2])
			self.request.joint_2_3_sv = float(sys.argv[3])
			self.request.time = float(sys.argv[4])
			self.req.method = sys.argv[5]
			self.future = self.client.call_async(self.req)
		except:
			pass
	#     except ValueError:
	#         self.req.position_joint1 = -1.
	#         self.req.position_joint2 = -1.
	#         self.req.position_joint3 = -1.
	#         self.req.oInterpolation_time = -1.
	#         self.req.method = ""
	#         self.future = self.client.call_async(self.req)
	#     except IndexError:
	#         self.req.position_joint1 = -1.
	#         self.req.position_joint2 = -1.
	#         self.req.position_joint3 = -1.
	#         self.req.oInterpolation_time = -1.
	#         self.req.method = ""
	#         self.future = self.client.call_async(self.req)

def main(args=None):
	rclpy.init(args=args)

	oint_client = Oint_client()
	oint_client.send_request()

	while rclpy.ok():
		rclpy.spin_once(oint_client)
		if oint_client.future.done():
			try:
				response = oint_client.future.result()
			except Exception as e:
				oint_client.get_logger().info(
					'Service call failed %r' % (e,))
			else:
				oint_client.get_logger().info(response.server_feedback)
			break

	oint_client.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()