import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist
import time

class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_node')
        self.scan_count = 0
        self.tf_count = 0
        self.odom_count = 0
        self.map_count = 0
        
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(TFMessage, '/tf', self.tf_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
    def scan_cb(self, msg):
        self.scan_count += 1
        
    def tf_cb(self, msg):
        self.tf_count += 1
        
    def odom_cb(self, msg):
        self.odom_count += 1

def main():
    rclpy.init()
    node = DebugNode()
    
    print("Listening to topics for 3 seconds...")
    end_time = time.time() + 3.0
    while time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)
        
    print(f"Scan messages received: {node.scan_count}")
    print(f"TF messages received: {node.tf_count}")
    print(f"Odom messages received: {node.odom_count}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
