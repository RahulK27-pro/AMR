import rclpy
from rclpy.node import Node
import time
import os

class TopicInspector(Node):
    def __init__(self):
        super().__init__('topic_inspector')
        self.output_file = '/home/usernamerahul/agv_ws/topic_debug.txt'
        
        # Clear file
        with open(self.output_file, 'w') as f:
            f.write("ROS 2 Topic Debug Report\n")
            f.write("========================\n")

    def run_tests(self):
        # 1. Get list of topics
        topics_and_types = self.get_topic_names_and_types()
        
        with open(self.output_file, 'a') as f:
            f.write("\nActive Topics:\n")
            for t_name, t_types in sorted(topics_and_types):
                f.write(f"- {t_name} : {t_types}\n")
                
def main():
    rclpy.init()
    node = TopicInspector()
    node.run_tests()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
