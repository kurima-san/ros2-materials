import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanMonitor(Node):
    def __init__(self):
        super().__init__('scan_monitor')
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.count = 0
    def on_scan(self, msg):
        self.count += 1
        valid=[x for x in msg.ranges if msg.range_min <= x <= msg.range_max]
        if valid and self.count % 10 == 0:
            self.get_logger().info(f'/scan samples={len(valid)} min={min(valid):.2f} m')

def main():
    rclpy.init(); node=ScanMonitor()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
