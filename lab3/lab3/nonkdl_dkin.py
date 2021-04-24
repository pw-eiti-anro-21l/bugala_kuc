from math import sin, cos, pi
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster, TransformStamped
from rclpy.clock import ROSClock
import time
from fk_solver import solve

class NonKdl(Node):


    def __init__(self):
        super().__init__('NonKdl_dkin')

        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):

    	T = solve(msg.position)
    	xyz = T.to_translation()
    	rpy = T.to_euler()
    	quater = rpy.to_quaternion()

    	pose_pub = self.create_publisher(PoseStamped, '/pose_stamped_nonkdl', QoSProfile(depth=10))

    	pose = PoseStamped()
    	now = self.get_clock().now()
    	pose.header.stamp = ROSClock().now().to_msg()
        pose.header.frame_id = "base"

        pose.pose.position.x = xyz[0]+0.1
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]+0.1
        pose.pose.orientation = Quaternion(w=quater[0], x=quater[1], y=quater[2], z=quater[3])
        pose_publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)

    nonkdl = NonKdl()

    rclpy.spin(nonkdl)

    nonkdl.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
	
    main()