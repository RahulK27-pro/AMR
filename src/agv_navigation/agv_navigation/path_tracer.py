import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import math

class PathTracerNode(Node):
    def __init__(self):
        super().__init__('path_tracer')
        
        # Subscribe to Odometry (Simulated Encoders)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Subscribe to IMU (Inner Ear)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        # Publisher for the continuous path trail
        self.path_pub = self.create_publisher(Path, '/agv_path', 10)
        
        # Initialize the Path object
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom' # The fixed frame of reference
        
        self.latest_yaw_rate = 0.0
        self.get_logger().info("Path Tracer Node Initialized: Tracking Wheel Encoders and IMU.")

    def imu_callback(self, msg):
        # The IMU gives us angular velocity in the Z axis (yaw rate).
        # We store this to compare against our wheel odometry for debugging slipping.
        self.latest_yaw_rate = msg.angular_velocity.z

    def odom_callback(self, msg):
        # 1. Create a single "Pose" (a snapshot of X, Y, and rotation)
        current_pose = PoseStamped()
        current_pose.header = msg.header
        current_pose.pose = msg.pose.pose
        
        # 2. Append this snapshot to our growing list of poses (The Path)
        self.path_msg.poses.append(current_pose)
        
        # 3. Publish the complete path to RViz
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)
        
        # Optional: Print to terminal to verify IMU and Odom are both active
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f'Traced Point -> X: {x:.2f}, Y: {y:.2f} | IMU Yaw Rate: {self.latest_yaw_rate:.2f} rad/s')

def main(args=None):
    rclpy.init(args=args)
    node = PathTracerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()