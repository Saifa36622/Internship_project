#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import os
from dynamixel_sdk import * 
from std_msgs.msg import Float32MultiArray,Int32,Float32
from gripper_state import GripperState

class servo(Node):
    def __init__(self):
        super().__init__('servo')
        
        # Control table addresses for X Series
        self.ADDR_OPERATING_MODE = 11
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_CURRENT = 102
        self.ADDR_PRESENT_CURRENT = 126
        self.ADDR_CURRENT_LIMIT = 38
        
        # Other parameters
        self.PROTOCOL_VERSION = 2.0
        self.DXL_ID = 0
        self.DEVICENAME = '/dev/ttyUSB0'
        self.BAUDRATE = 57600
        
        # Current values (adjusted for X series)
        # For X series, current value of 1 equals 2.69 mA
        # 500 mA = 186 units
        self.dxl_goal_current = [0, 500, -500]  # 0mA, 500mA, -500mA
        
        # Initialize SDK
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        
        # Setup the servo
        if not self.setup_servo():
            self.get_logger().error("Failed to setup servo")
            return
            
        # Create ROS2 timer and subscription
        self.create_timer(0.1, self.timer_callback)
        self.create_subscription(Int32, "/servo_cmd", self.servo_callback, 10)
        
    def setup_servo(self):
        # Open port
        if not self.portHandler.openPort():
            self.get_logger().error("Failed to open port")
            return False
            
        # Set baudrate
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            self.get_logger().error("Failed to set baudrate")
            return False
            
        # Disable torque first (required to change modes)
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, 0)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            self.get_logger().error("Failed to disable torque")
            return False
            
        # Set to Current Control Mode (0)
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_OPERATING_MODE, 0)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            self.get_logger().error("Failed to set Current Control Mode")
            return False
            
        # Set Current Limit (500mA = 186 units)
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_CURRENT_LIMIT, 186)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            self.get_logger().error("Failed to set current limit")
            return False
            
        # Enable torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.DXL_ID, self.ADDR_TORQUE_ENABLE, 1)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            self.get_logger().error("Failed to enable torque")
            return False
            
        self.get_logger().info("Servo setup completed successfully")
        return True
        
    def timer_callback(self):
        # Read present current
        dxl_present_current, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(
            self.portHandler, 
            self.DXL_ID, 
            self.ADDR_PRESENT_CURRENT
        )
        
        if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
            # Convert to mA for better understanding
            current_ma = dxl_present_current * 2.69
            self.get_logger().info(f"[ID:{self.DXL_ID:03d}] Present Current: {dxl_present_current} ({current_ma:.2f} mA)")

    def servo_callback(self, msg: Int32):
        # msg.data: 0 = stop, 1 = CW, 2 = CCW
        if msg.data not in [0, 1, 2]:
            self.get_logger().error("Invalid command. Use 0 for stop, 1 for CW, 2 for CCW")
            return
            
        # Update gripper state
        GripperState.set_current(msg.data)
        
        # Get current value and apply
        current_value = self.dxl_goal_current[msg.data]
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(
            self.portHandler, 
            self.DXL_ID, 
            self.ADDR_GOAL_CURRENT, 
            current_value if current_value >= 0 else (0xFFFF + current_value + 1)  # Convert negative values
        )
        
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            self.get_logger().error("Failed to set current")
            return
            
        direction = "STOP" if msg.data == 0 else "CW" if msg.data == 1 else "CCW"
        current_ma = current_value * 2.69
        self.get_logger().info(f"Setting current to {current_value} ({current_ma:.2f} mA) - {direction}")
    
    def cleanup(self):
        """Clean up before shutdown"""
        if hasattr(self, 'portHandler') and self.portHandler.is_open:
            # Disable torque
            self.packetHandler.write1ByteTxRx(
                self.portHandler, 
                self.DXL_ID, 
                self.ADDR_TORQUE_ENABLE, 
                0
            )
            self.portHandler.closePort()

def main(args=None):
    rclpy.init(args=args)
    node = servo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.cleanup()
            node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()