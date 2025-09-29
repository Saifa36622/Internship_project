#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from rtde_receive import RTDEReceiveInterface
import json
import time
import os

class JointServoLogger(Node):
    def __init__(self):
        super().__init__('joint_servo_logger')
        
        # Initialize RTDE receive interface for UR robot
        self.rtde_receive = RTDEReceiveInterface("192.168.12.60")
        
        # Subscribe to servo command topic
        self.servo_subscription = self.create_subscription(
            Int32,
            '/servo_cmd',
            self.servo_callback,
            10
        )
        
        # Initialize state variables
        self.servo_state = 0  # Default state when no commands received
        self.joint_positions = [0.0] * 6  # Initialize with zeros
        
        # Create timer for periodic joint position updates
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz update rate
        
        # JSON file path
        self.json_file = 'robot_state.json'
        
        self.get_logger().info('Joint and Servo Logger Node started')

    def servo_callback(self, msg):
        """Handle received servo commands"""
        self.servo_state = msg.data
        self.update_json_file()

    def timer_callback(self):
        """Periodic callback to update joint positions"""
        try:
            # Get current joint positions
            self.joint_positions = self.rtde_receive.getActualQ()
            self.update_json_file()
        except Exception as e:
            self.get_logger().error(f'Error getting joint positions: {str(e)}')

    def update_json_file(self):
        """Update the JSON file with current state by appending new data"""
        state_data = {
            'timestamp': time.time(),
            'joint_positions': self.joint_positions,
            'servo_state': self.servo_state
        }
        
        try:
            # Read existing data
            existing_data = []
            if os.path.exists(self.json_file):
                try:
                    with open(self.json_file, 'r') as f:
                        existing_data = json.load(f)
                except json.JSONDecodeError:
                    # If file is empty or invalid JSON, start with empty list
                    existing_data = []
                
                # Ensure existing_data is a list
                if not isinstance(existing_data, list):
                    existing_data = [existing_data]
            
            # Append new data
            existing_data.append(state_data)
            
            # Write back all data
            with open(self.json_file, 'w') as f:
                json.dump(existing_data, f, indent=4)
        except Exception as e:
            self.get_logger().error(f'Error writing to JSON file: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = JointServoLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()