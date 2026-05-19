import rclpy
from rclpy.node import Node
import subprocess

def main():
    rclpy.init()
    node = rclpy.create_node('diag_node')
    
    with open('/tmp/diag_report.txt', 'w') as f:
        f.write("=== NODES ===\n")
        f.write("\n".join(node.get_node_names()))
        f.write("\n\n=== TOPICS ===\n")
        for topic_name, topic_types in node.get_topic_names_and_types():
            f.write(f"{topic_name}: {topic_types}\n")
            
        f.write("\n=== LIFECYCLE STATES ===\n")
        
    node.destroy_node()
    rclpy.shutdown()

    # Get lifecycle states using ros2cli
    with open('/tmp/diag_report.txt', 'a') as f:
        nodes = ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother', 'smoother_server', 'map_saver']
        for n in nodes:
            try:
                res = subprocess.run(['ros2', 'lifecycle', 'get', f'/{n}'], capture_output=True, text=True, timeout=2)
                f.write(f"/{n}: {res.stdout.strip()} {res.stderr.strip()}\n")
            except Exception as e:
                f.write(f"/{n}: Exception {e}\n")

if __name__ == '__main__':
    main()
