#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from dynamixel_sdk import * # Dynamixel SDK
from std_msgs.msg import Int32

# Control table address
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102
ADDR_CURRENT_LIMIT = 38 # Address for the Current Limit

# Protocol version
PROTOCOL_VERSION = 2.0

# Default setting
DXL_ID = 0  # Dynamixel ID
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'  # Check your port name

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
CURRENT_LIMIT_CW = 500  # Safe value for clockwise rotation
CURRENT_LIMIT_CCW = -500  # Safe value for counter-clockwise rotation

class DynamixelCurrentControl(Node):

    def __init__(self):
        super().__init__('dynamixel_current_control_node')
        self.get_logger().info("Dynamixel Current Control Node Started")

        # Set up Dynamixel port and packet handlers
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Open port and set baudrate
        if not self.portHandler.openPort():
            self.get_logger().error("Failed to open the port!")
            self.destroy_node()
            return

        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set the baudrate!")
            self.destroy_node()
            return

        # Check the servo's current limit to avoid the error
        dxl_current_limit, _, _ = self.packetHandler.read2ByteTxRx(self.portHandler, DXL_ID, ADDR_CURRENT_LIMIT)
        self.get_logger().info(f"Dynamixel's Current Limit is: {dxl_current_limit}")
        
        # Initialize the servo
        self.setup_servo()

        # Create subscription
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            Int32,
            '/servo_cmd',
            self.servo_callback,
            qos_profile
        )
        self.get_logger().info("Subscription to '/servo_cmd' topic created.")

    def setup_servo(self):
        # Disable torque before changing operating mode
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.get_logger().info("Torque disabled.")

        # Set operating mode to Current Control Mode (0)
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_OPERATING_MODE, 0)
        self.get_logger().info("Operating mode set to Current Control.")

        # Enable torque
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        self.get_logger().info("Torque enabled.")

    def servo_callback(self, msg):
        command = msg.data
        goal_current = 0

        if command == 1:
            goal_current = CURRENT_LIMIT_CW
        elif command == 2:
            goal_current = CURRENT_LIMIT_CCW
        elif command == 0:
            goal_current = 0
        else:
            self.get_logger().warn(f"Invalid command received: {command}. Ignoring.")
            return

        # Write the Goal Current value
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(
            self.portHandler, DXL_ID, ADDR_GOAL_CURRENT, goal_current
        )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Comm error: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            self.get_logger().error(f"Dxl error: {self.packetHandler.getRxPacketError(dxl_error)}")
        else:
            self.get_logger().info(f"Successfully set Goal Current to: {goal_current}")

    def on_shutdown(self):
        # Disable torque on shutdown
        self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.get_logger().info("Torque disabled. Shutting down.")
        self.portHandler.closePort()

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelCurrentControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()