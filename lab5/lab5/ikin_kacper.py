import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from math import cos, sin, atan, atan2, sqrt, acos, asin, pi
from rclpy.qos import QoSProfile
import transformations
import mathutils

class Ikin(Node):

	def __init__(self):
		super().__init__('ikin')
		self.srv = self.create_service(InvKin, 'InvKin', self.ikin_callback)
		qos_profile = QoSProfile(depth=10)
		self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
		self.pose_sub = self.create_subscription(PoseStamped, 'ikin_pose', self.listener_callback, qos_profile)
		self.base_height = get_params('link_0', 'links_xyz.json')['length']
		self.length_1_2 = get_params('link_2', 'links_xyz.json')['length']
		self.length_2_tool = get_params('link_3', 'links_xyz.json')['length'] + get_params('link_4', 'links_xyz.json')['length']

		
		self.starting_position =  self.initial_position
	
	def get_params(part, filename):
	    with open(filename, "r") as file:
	        read_file = json.load(file)
	    part_params = read_file[part]
	    return part_params

	def ikin_callback(self, request, response):
		if request.time > 0:

			if request.method == "rectangle":
				self.rectangle_ip(request)
				response.output = "Interpolation finished with linear method."

			elif request.method == "ellipse":
				self.ellipse_ip(request)
				response.output == "oInterpolation finished with trapezoid method."
			
			else:
				response.output == "This method does not exist."
		else:
			response.output = "Error! Value of time is invalid."

		return response

	def listener_callback(self, msg):
		for i in range(3):
			self.initial_position[i] = msg.position[i]

	def rectangle_ip(self, request):
		sample_time = 0.01
		a = request.a
		b = request.b
		self.current_goal = []
		i = 0
		steps = floor(request.time/sample_time)

		joint_states = JointState()
		joint_states.name = ['joint_0_1', 'joint_1_2', 'joint_2_3']
		current_joint_states = self.initial_position


		while (true):
			current_position = self.initial_position
			current_orientation = self.initial_orientation
			for step in range(1, steps+1):	
				if (i==0):
					self.current_goal = [self.starting_position[0],
							self.starting_position[1]+(a/2)*step/steps,
							self.starting_position[2]]



				elif (i==1):
					self.current_goal = [self.starting_position[0],
							self.starting_position[1]+(a/2),
							self.starting_position[2]+b*step/steps]

				elif (i==2):
					self.current_goal = [self.starting_position[0],
							self.starting_position[1]+(a/2)-(a)*step/steps,
							self.starting_position[2]+b]

				elif (i==3):
					self.current_goal = [self.starting_position[0],
							self.starting_position[1]-(a/2),
							self.starting_position[2]+b-b*step/steps]

				elif (i==4):
					self.current_goal = [self.starting_position[0],
							self.starting_position[1]-(a/2)+(a/2)*step/steps,
							self.starting_position[2]]

				req_joints = find_joint_states(self.current_goal)
				
				joint_0_1_state = current_joint_states[0] + (req_joints[0] - current_joint_states[0])/steps*step
				joint_1_2_state = current_joint_states[1] + (req_joints[1] - current_joint_states[1])/steps*step
				joint_2_3_state = current_joint_states[2] + (req_joints[2] - current_joint_states[2])/steps*step
				joint_states.position = [float(joint_0_1_state), float(joint_1_2_state), float(joint_2_3_state)]

				self.joint_pub.publish(joint_states)
				sleep(sample_time)

			self.initial_position = [joint_0_1_state, joint_1_2_state, joint_2_3_state]


	



	def find_joint_states(self, point):
		x = point[0]
		y = point[1]
		z = point[2] - self.base_height
		a = self.length_1_2
		d = self.length_2_tool
		dist = sqrt(x*x + y*y + z*z)
		gamma = acos((a*a + d*d - dist*dist)/(2*a*d))
		joint_2_3 = pi - gamma
		alpha = asin(d*sin(gamma)/dist)
		joint_1_2 = -(alpha + atan(z/sqrt(x*x + y*y)))
		joint_0_1 = atan(y/x)
		return [joint_0_1, joint_1_2, joint_2_3]





def main(args=None):
	rclpy.init(args=args)
	ikin = Ikin()
	rcply.spin(ikin)
	rclpy.shutdown()

if __name__ == '__main__':
	main()

