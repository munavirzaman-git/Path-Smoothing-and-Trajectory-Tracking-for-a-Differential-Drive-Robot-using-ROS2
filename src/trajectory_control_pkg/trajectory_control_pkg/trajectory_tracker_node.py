import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
from tf_transformations import euler_from_quaternion
from .pure_pursuit import pure_pursuit_control
import numpy as np
import csv

class TrajectoryTracker(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')

        self.traj = []
        self.pose = None
        self.cte_list = []

        self.create_subscription(Path, '/trajectory', self.traj_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.actual_path_pub = self.create_publisher(Path, '/actual_path', 10)
        self.lookahead_pub = self.create_publisher(Marker, '/lookahead_point', 10)

        self.actual_path = Path()
        self.actual_path.header.frame_id = 'odom'

        self.timer = self.create_timer(0.1, self.control_loop)

    def traj_callback(self, msg):
        self.traj = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            t = pose.header.stamp.sec
            self.traj.append((x, y, t))

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose = (x, y, yaw)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation = msg.pose.pose.orientation

        self.actual_path.poses.append(pose)
        self.actual_path_pub.publish(self.actual_path)

    def control_loop(self):
        if self.pose is None or not self.traj:
            return

        v, omega, target = pure_pursuit_control(self.pose, self.traj)

        # CTE calculation
        min_dist = float('inf')
        for px, py, _ in self.traj:
            dist = np.hypot(px - self.pose[0], py - self.pose[1])
            if dist < min_dist:
                min_dist = dist
        self.cte_list.append(min_dist)

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

        # Publish lookahead marker
        if target is not None:
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.type = Marker.SPHERE
            marker.scale.x = 0.12
            marker.scale.y = 0.12
            marker.scale.z = 0.12
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.pose.position.x = target[0]
            marker.pose.position.y = target[1]
            marker.pose.position.z = 0.0
            self.lookahead_pub.publish(marker)

    def destroy_node(self):
        with open('/tmp/cte_log.csv', 'w') as f:
            writer = csv.writer(f)
            for e in self.cte_list:
                writer.writerow([e])
        super().destroy_node()

def main():
    rclpy.init()
    node = TrajectoryTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

