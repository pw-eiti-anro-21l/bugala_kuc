from lab4_interfaces.srv import oInterpolation
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from math import cos, sin, floor, pi
from rclpy.qos impot QoSProfile
from time import sleep


class Oint(Node):

    def __init__(self):
        super().__init__('oint')
        self.srv = self.create_service(oInterpolation, 'oInterpolation', self.oInterpolation_callback)
        qos_profile = QoSProfile(depth=10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/marker', qos_profile)
        self.pose_publisher = self.create_publisher(PoseStamped, 'oint_pose', qos_profile)
        self.initial_position = [0, 0, 0]
        self.initial_orientation = [0, 0, 0]

	def oInterpolation_callback(self, request, response):
		if request.time > 0:
            #wrocimy tu

			if request.method == "linear":
				self.linear_ip(request)
				response.output = "Linear oInterpolation completed."

			elif request.method == "trapezoid":
				self.trapezoid_ip(request):
				response.output == "Trapezoid oInterpolation completed."
			
			else:
				response.output == "Wrong method name."
		else:
			response.output = "Error! Wrong value of time."

		return response

	def linear_ip(self, request):
        sample_time = 0.01
        steps = floor(request.time/sample_time)
        pose = PoseStamped()
        current_position = self.initial_position
        current_orientation = self.initial_orientation
        initial_position = current_position
        initial_orientation = current_orientation

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
            current_position[0] += (request.x_sv - initial_position[0])/steps
            current_position[1] += (request.y_sv - initial_position[1])/steps
            current_position[2] += (request.z_sv - initial_position[2])/steps
            
            current_orientation[0] += (request.roll_sv - initial_orientation[0])/steps
            current_orientation[1] += (request.pitch_sv - initial_orientation[1])/steps
            current_orientation[2] += (request.yaw_sv - initial_orientation[2])/steps
            orientation_quaternion = Quaternion(w=0.0, x=current_orientation[0], y=current_orientation[1], z=current_orientation[2])


            if request.version == "ext":  
                orientation_quaternion = self.euler_to_quaternion(current_orientation[0], current_orientation[1], current_orientation[2])
            else:
                orientation_quaternion = self.euler_to_quaternion(0, 0, 0)

            pose.header.frame_id = "base"
            pose.pose.position.x = current_position[0]
            pose.pose.position.y = current_position[1]
            pose.pose.position.z = current_position[2]
            pose.pose.orientation = orientation_quaternion
            sleep(sample_time)
            
            self.pose_publisher.publish(pose)
            marker.pose.position.x = current_position[0]
            marker.pose.position.y = current_position[1]
            marker.pose.position.z = current_position[2]
            marker.pose.orientation = orientation_quaternion
            markers.markers.append(marker)
            id=0
            for marker in markers.markers:
                marker.id = id
                id += 1
            self.marker_publisher.publish(markers)

        self.initial_position = current_position
        self.initial_orientation = current_orientation

    def trapezoid_ip(self, request):
        sample_time = 0.01
        steps = floor(request.oInterpolation_time/sample_time)
        pose = PoseStamped()
        current_position = self.initial_position
        current_orientation = self.initial_orientation
        initial_position = current_position
        initial_orientation = current_orientation

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

            if request.version == "ext":  
                ort_quaternion = self.euler_to_quaternion(current_orientation[0], current_orientation[1], current_orientation[2])
            else:
                ort_quaternion = self.euler_to_quaternion(0, 0, 0)
            
            pose.header.frame_id = "base"
            pose.pose.position.x = current_position[0]
            pose.pose.position.y = current_position[1]
            pose.pose.position.z = current_position[2]
            pose.pose.orientation = orientation_quaternion
            sleep(sample_time)
            
            self.pose_publisher.publish(pose)
            marker.pose.position.x = current_position[0]
            marker.pose.position.y = current_position[1]
            marker.pose.position.z = current_position[2]
            marker.pose.orientation = orientation_quaternion
            markers.markers.append(marker)
            id=0
            for marker in markers.markers:
                marker.id = id
                id += 1
            self.marker_publisher.publish(markers)

        self.initial_position = current_position
        self.initial_orientation = current_orientation


    def euler_to_quaternion(self,roll, pitch, yaw):
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)

    oint = Oint()

    rclpy.spin(oint)

    rclpy.shutdown()


if __name__ == '__main__':
    main()