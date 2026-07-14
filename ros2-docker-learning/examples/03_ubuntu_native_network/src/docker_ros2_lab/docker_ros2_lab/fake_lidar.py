import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class FakeLidar(Node):
    def __init__(self):
        super().__init__('fake_lidar')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.phase = 0.0
        self.timer = self.create_timer(0.1, self.publish_scan)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'; tf.child_frame_id = 'laser'
        tf.transform.rotation.w = 1.0
        self.tf = StaticTransformBroadcaster(self)
        self.tf.sendTransform(tf)
        self.get_logger().info('Publishing hardware-free LaserScan on /scan')

    def publish_scan(self):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = 'laser'
        msg.angle_min = -math.pi; msg.angle_max = math.pi
        msg.angle_increment = math.pi / 180.0
        msg.range_min = 0.1; msg.range_max = 10.0
        msg.scan_time = 0.1; msg.time_increment = msg.scan_time / 360.0
        ranges=[]
        for i in range(360):
            a = msg.angle_min + i * msg.angle_increment
            wall = 4.0 + 0.4 * math.sin(3.0*a + self.phase)
            obstacle = 1.2 if abs(a - 0.45*math.sin(self.phase)) < 0.12 else wall
            ranges.append(float(obstacle))
        msg.ranges = ranges
        self.pub.publish(msg); self.phase += 0.08

def main():
    rclpy.init(); node=FakeLidar()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
