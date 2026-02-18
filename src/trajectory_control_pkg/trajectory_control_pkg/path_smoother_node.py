import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from .spline_utils import smooth_path

class PathSmoother(Node):
    def __init__(self):
        super().__init__('path_smoother')

        self.pub = self.create_publisher(Path, '/smoothed_path', 10)
        self.timer = self.create_timer(2.0, self.publish_path)

        self.waypoints = [(0,0), (1,0), (2,1), (3,1), (4,0)]
        #self.waypoints = [(0,0), (1,0), (2,0), (3,0), (4,0)]




    def publish_path(self):
        smooth = smooth_path(self.waypoints)

        path_msg = Path()
        path_msg.header.frame_id = 'odom'

        for x, y in smooth:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.pub.publish(path_msg)

def main():
    rclpy.init()
    node = PathSmoother()
    rclpy.spin(node)
    rclpy.shutdown()

