import sys
import rclpy
from rclpy.node import Node
from lab4_interfaces.srv import OInterpolation as oInterpolation 

class Oint_client(Node):
	def __init__(self):
		super().__init__('oint_client')
		self.client = self.create_client(oInterpolation, 'oInterpolation')
		while not self.client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Service not available, waiting again...')
		self.request = oInterpolation.Request()

	def send_request(self):
		self.request.x_sv = float(sys.argv[1])
		self.request.y_sv = float(sys.argv[2])
		self.request.z_sv = float(sys.argv[3])
		self.request.roll_sv = float(sys.argv[4])
		self.request.pitch_sv = float(sys.argv[5])
		self.request.yaw_sv = float(sys.argv[6])
		self.request.time = float(sys.argv[7])
		self.request.method = sys.argv[8]
		self.request.version = sys.argv[9]
		self.future = self.client.call_async(self.request)

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