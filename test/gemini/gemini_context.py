#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from google import genai
from PIL import Image
import os
from std_msgs.msg import Int32
import cv2  # Import OpenCV library
import numpy as np  # Import numpy for image manipulation


class GeminiContextNode(Node):
    def __init__(self):
        super().__init__('gemini_context_node')
        self.subscription = self.create_subscription(
            String,
            '/gemini/context',
            self.context_callback,
            10)
        self.publisher = self.create_publisher(Int32, '/start', 10) 
        # TODO: define the allowed commands for clarity in the prompt
        self.allowed_commands = ["", "", ""]
        self.model_name = "gemini-2.5-flash"
        self.client = genai.Client()
        self.context_file = os.path.expanduser("~/main_ws/gemini_context.json")
        self.get_logger().info("GeminiContextNode initialized and ready.")

    def context_callback(self, msg: String):
        try:
            # Load the image for context
            img = Image.open(os.path.expanduser("~/main_ws/test-img.jpg"))
             # Capture image from the CV camera
            # cap = cv2.VideoCapture(4)  # 0 is usually the default camera index
            # if not cap.isOpened():
            #     self.get_logger().error("Error: Could not open video camera.")
            #     return

            # ret, frame = cap.read()
            # cap.release()  # Release the camera immediately after capturing the frame
            
            # if not ret:
            #     self.get_logger().error("Error: Failed to capture image from camera.")
            #     return

            # # Convert the OpenCV image (numpy array) to a PIL Image object
            # frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # img = Image.fromarray(frame_rgb)
            # img.show()  # Display the captured image for verification
            self.get_logger().info(f"Processing task: {msg.data} with image context")

            # Prepare the content with image context
            # TODO: Add example json prompt
            content = f"""Right now I am trying to use you as a decision maker for my manipulation robot.
            # For the task: "{msg.data}", analyze the task including action and sub action that not directly in the input but appropriate from the scence context image 
            # and then respond in JSON format.The JSON should contain a single key, "steps", which holds a list of dictionaries.
            # Each dictionary in the list should have two keys: "command" and "object".
            # The "command" must be one of the following: {self.allowed_commands}.

            # Example of desired JSON format:
            # {{
            #   "steps": [
            #     {{"command": "", "object": ""}},
            #     {{"command": "", "object": ""}},
            #     {{"command": "", "object": ""}}
            #   ]
            # }}"""

            # Generate response from Gemini
            response = self.client.models.generate_content(
                model=self.model_name,
                contents=[img, content]
            )

            self.get_logger().info("Raw response from Gemini:")
            self.get_logger().info(response.text)

            # Clean up the response text
            json_string = response.text.strip()
            if json_string.startswith("```json"):
                json_string = json_string[len("```json"):].strip()
            if json_string.endswith("```"):
                json_string = json_string[:-len("```")].strip()

            # Parse and save response
            parsed_data = json.loads(json_string)

            # Save to file
            with open(self.context_file, 'w') as f:
                json.dump(parsed_data, f, indent=2)

            self.get_logger().info(f"Saved Gemini context to {self.context_file}")
            self.publisher.publish(Int32(data=1))

        except FileNotFoundError:
            self.get_logger().error("Error: Image file not found")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error decoding JSON: {e}\nProblematic text: {response.text}")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")

# # Construct the prompt to clearly instruct Gemini on the desired JSON format
# # The prompt is similar to the original, but the robot_input is updated.
# content = f"""Right now I am trying to use you as a decision maker for my mobile manipulation robot.
# For the task: "{robot_input}", analyze the task including action and sub action that not directly in the input but appropriate from the scence context image 
# and then respond in JSON format.The JSON should contain a single key, "steps", which holds a list of dictionaries.
# Each dictionary in the list should have two keys: "command" and "object".
# The "command" must be one of the following: {allowed_commands}.

# Example of desired JSON format:
# {{
#   "steps": [
#     {{"command": "move_to", "object": "water bottle"}},
#     {{"command": "pick_up", "object": "water bottle"}},
#     {{"command": "place", "object": "water bottle"}}
#   ]
# }}
# """

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GeminiContextNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error creating node: {e}")
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()