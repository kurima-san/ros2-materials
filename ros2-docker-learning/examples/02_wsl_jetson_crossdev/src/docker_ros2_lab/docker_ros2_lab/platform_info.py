import os, platform
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class PlatformInfo(Node):
    def __init__(self):
        super().__init__('platform_info')
        self.pub=self.create_publisher(String,'/platform_info',10)
        self.timer=self.create_timer(2.0,self.publish)
    def publish(self):
        value=f"machine={platform.machine()} system={platform.system()} target={os.getenv('TARGET_DEVICE','unknown')}"
        self.pub.publish(String(data=value)); self.get_logger().info(value)
def main():
    rclpy.init(); n=PlatformInfo()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()
