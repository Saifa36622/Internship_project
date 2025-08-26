#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import json
import os

class GeminiCmdPrinter(Node):
    def __init__(self):
        super().__init__('gemini_cmd_printer')
        self.json_file = os.path.expanduser('~/main_ws/gemini_cmd.json')
        self.create_timer(1.0, self.timer_callback)  # Check every second
        self.printed = False

    def timer_callback(self):
        if self.printed:
            return
        if not os.path.exists(self.json_file):
            self.get_logger().info(f"Waiting for {self.json_file} to be created...")
            return
        try:
            with open(self.json_file, 'r') as f:
                data = json.load(f)
            if 'steps' in data and isinstance(data['steps'], list):
                self.get_logger().info("Printing Gemini command steps:")
                for step in data['steps']:
                    cmd = step.get('command', 'N/A')
                    obj = step.get('object', 'N/A')
                    print(f"Command: {cmd}, Object: {obj}")
                self.printed = True
            else:
                self.get_logger().warn("No 'steps' key or not a list in JSON file.")
        except Exception as e:
            self.get_logger().error(f"Error reading or parsing JSON: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = GeminiCmdPrinter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
