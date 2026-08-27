import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')
        self.img_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # Publish the obstacle's location. 
        # x = Horizontal position (-1.0 to 1.0), y = Proximity (Bounding Box Size)
        self.alert_pub = self.create_publisher(Point, '/obstacle_alert', 10)
        
        self.bridge = CvBridge()

        # GUI display — auto-detect if a display server is available (X11/Wayland)
        self.declare_parameter('show_gui', True)
        self.show_gui = self.get_parameter('show_gui').get_parameter_value().bool_value
        if self.show_gui and not os.environ.get('DISPLAY'):
            self.get_logger().warn("No DISPLAY environment variable set. Disabling GUI window.")
            self.show_gui = False

        self.get_logger().info("Edge-AI Vision Node Active. Scanning for Obstacles...")

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, _ = cv_image.shape
            
            # --- Color Detection for Red Obstacles ---
            # Red wraps around the HSV hue wheel, so we need TWO ranges
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Range 1: low red hues (H: 0-10)
            lower_red1 = np.array([0, 120, 70])
            upper_red1 = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

            # Range 2: high red hues (H: 170-180) — the wraparound
            lower_red2 = np.array([170, 120, 70])
            upper_red2 = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

            # Combine both ranges
            mask = mask1 | mask2
            
            # Find the obstacle's bounding box
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            alert_msg = Point()
            alert_msg.x = 0.0 # 0 means path is clear
            alert_msg.y = 0.0 # 0 means no obstacle
            
            if contours:
                # Find the largest obstacle
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 1000: # Ignore tiny specks of noise
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    
                    # Calculate center of obstacle relative to the camera center
                    # -1.0 means far left of screen, 1.0 means far right
                    center_x = x + (w / 2)
                    screen_center = width / 2
                    offset_x = (center_x - screen_center) / screen_center
                    
                    alert_msg.x = float(offset_x)
                    alert_msg.y = float(area) # Use area as a proxy for how close it is
                    
                    # Draw a green bounding box for visualization
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.putText(cv_image, f"OBSTACLE! Offset: {offset_x:.2f}", (x, y-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            self.alert_pub.publish(alert_msg)
            
            # Show the Dashcam View (only if display is available)
            if self.show_gui:
                cv2.imshow("AGV Dashcam - AI Vision", cv_image)
                cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Vision Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()