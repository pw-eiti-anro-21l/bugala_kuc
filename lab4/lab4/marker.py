import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Pose, PoseStamped
from math import sin, cos
from rclpy.clock import ROSClock
from visualization_msgs.msg import MarkerArray, Marker

class Marker(Node):

    def __init__(self):
        super().__init__('marker')
        self.subscription = self.create_subscription(
            PoseStamped,
            'oint_pub',
            self.listener_callback, 1)
        self.marker_pub = self.create_publisher(MarkerArray, "trajectory_point", 1)
        self.markers = MarkerArray()

    def listener_callback(self, msg):
        pose = PoseStamped()
        pose.pose = msg.pose
        pose.header.stamp = ROSClock().now().to_msg()
        pose.header.frame_id = "tool"

        marker = Marker()
        marker.header.stamp = ROSClock().now().to_msg()
        marker.header.frame_id = "/tool"
        marker.type = 2
        marker.pose = pose_st.pose
        marker.id = len(self.markers.markers) + 1
        marker.action = Marker.ADD
        marker.type = Marker.SPHERE
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 0.5
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        self.markers.markers.append(marker)
        self.publisher2.publish(self.markers)
        print(self.markers.markers)




def main(args=None):
    rclpy.init(args=args)

    trajectory = Marker()

    rclpy.spin(trajectory)
    trajectory.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()