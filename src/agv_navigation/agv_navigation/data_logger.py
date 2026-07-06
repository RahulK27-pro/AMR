import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import os
import csv
import time
from datetime import datetime

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Latest data
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.min_obstacle_dist = float('inf')
        
        # Setup CSV Logging
        log_dir = os.path.join(os.environ['HOME'], 'AMR', 'logs')
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
            
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file_path = os.path.join(log_dir, f'robot_log_{timestamp}.csv')
        
        self.file = open(self.log_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.file)
        self.csv_writer.writerow(['Timestamp', 'X', 'Y', 'Yaw', 'Cmd_V', 'Cmd_W', 'Min_Obstacle_Dist'])
        
        self.get_logger().info(f"Logging data to: {self.log_file_path}")
        
        # Log data at 10 Hz
        self.log_timer = self.create_timer(0.1, self.log_data)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = self.get_yaw_from_quaternion(msg.pose.pose.orientation)

    def cmd_callback(self, msg):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        
    def scan_callback(self, msg):
        ranges = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if ranges:
            self.min_obstacle_dist = min(ranges)
        else:
            self.min_obstacle_dist = float('inf')

    def log_data(self):
        current_time = time.time()
        self.csv_writer.writerow([
            f"{current_time:.3f}", 
            f"{self.current_x:.4f}", 
            f"{self.current_y:.4f}", 
            f"{self.current_yaw:.4f}", 
            f"{self.cmd_v:.4f}", 
            f"{self.cmd_w:.4f}", 
            f"{self.min_obstacle_dist:.4f}"
        ])
        self.file.flush()
        
    def destroy_node(self):
        self.get_logger().info("Closing log file.")
        self.file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
