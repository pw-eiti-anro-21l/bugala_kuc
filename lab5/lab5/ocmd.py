import sys
import rclpy
from rclpy.node import Node
from lab5_interfaces.srv import InvKin 

class Oint_client(Node):
	def __init__(self):
		super().__init__('oint_client')
		self.client = self.create_client(InvKin, 'invkin')
		while not self.client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service not available, waiting again...')
		self.request = InvKin.Request()

	def send_request(self):
		try:
			self.request.a = float(sys.argv[1])
			self.request.b = float(sys.argv[2])
			self.request.time = float(sys.argv[3])
			self.request.method = sys.argv[4]
			self.future = self.client.call_async(self.request)
		except ValueError:
			self.get_logger().info("Incorrect parameters")
		except IndexError:
			self.get_logger().info("Not enough parameters")

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
				oint_client.get_logger().info(response.output)
			break

	oint_client.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()