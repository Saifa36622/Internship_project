#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from google import genai
import os

class GeminiNode(Node):
    def __init__(self):
        super().__init__('gemini_node')
        self.subscription = self.create_subscription(
            String,
            '/gemini/task',
            self.task_callback,
            10)
        self.allowed_commands = ["move_to", "pick_up", "place"]
        self.model_name = "gemini-2.5-flash"
        self.client = genai.Client()
        self.output_file = os.path.expanduser("~/main_ws/gemini_cmd.json")
        self.get_logger().info("GeminiNode initialized and ready.")

    def task_callback(self, msg):
        robot_input = msg.data
        content = f"""Right now I am trying to use you as a decision maker for my mobile manipulation robot.\nFor the task: \"{robot_input}\", analyze the task and then respond in JSON format.\nThe JSON should contain a single key, \"steps\", which holds a list of dictionaries.\nEach dictionary in the list should have two keys: \"command\" and \"object\".\nThe \"command\" must be one of the following: {self.allowed_commands}.\n\nExample of desired JSON format:\n{{\n  \"steps\": [\n    {{\"command\": \"move_to\", \"object\": \"water bottle\"}},\n    {{\"command\": \"pick_up\", \"object\": \"water bottle\"}},\n    {{\"command\": \"place\", \"object\": \"water bottle\"}}\n  ]\n}}\n"""
        try:
            response = self.client.models.generate_content(
                model=self.model_name,
                contents=content
            )
            self.get_logger().info("Raw response from Gemini:")
            self.get_logger().info(response.text)
            json_string = response.text.strip()
            if json_string.startswith("```json"):
                json_string = json_string[len("```json"):].strip()
            if json_string.endswith("```"):
                json_string = json_string[:-len("```")].strip()
            parsed_data = json.loads(json_string)
            # Save to file
            with open(self.output_file, 'w') as f:
                json.dump(parsed_data, f, indent=2)
            self.get_logger().info(f"Saved Gemini command JSON to {self.output_file}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding JSON: {e}\nProblematic text: {response.text}")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = GeminiNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
