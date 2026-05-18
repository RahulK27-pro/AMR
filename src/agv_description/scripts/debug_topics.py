import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = rclpy.create_node('debug_topics')
    print("=== NODES ===")
    print("\n".join(node.get_node_names()))
    print("\n=== TOPICS ===")
    for topic_name, topic_types in node.get_topic_names_and_types():
        print(f"{topic_name}: {topic_types}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
