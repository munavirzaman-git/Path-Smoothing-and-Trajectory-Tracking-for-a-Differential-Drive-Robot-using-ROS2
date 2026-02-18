import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import numpy as np

class TrajectoryGenerator(Node):
    def __init__(self):
        super().__init__('trajectory_generator')

        self.sub = self.create_subscription(Path, '/smoothed_path', self.callback, 10)
        self.pub = self.create_publisher(Path, '/trajectory', 10)

    def callback(self, msg):
        traj = Path()
        traj.header.frame_id = 'odom'
        traj.header.stamp = self.get_clock().now().to_msg()

        t = 0.0
        velocity = 0.25

        for i in range(len(msg.poses)):
            pose = msg.poses[i]

            pose.header.frame_id = 'odom'
            pose.header.stamp = self.get_clock().now().to_msg()

            if i > 0:
                prev = msg.poses[i-1]
                dx = pose.pose.position.x - prev.pose.position.x
                dy = pose.pose.position.y - prev.pose.position.y
                dist = np.hypot(dx, dy)
                t += dist / velocity

            traj.poses.append(pose)

        self.pub.publish(traj)

def main():
    rclpy.init()
    node = TrajectoryGenerator()
    rclpy.spin(node)
    rclpy.shutdown()

