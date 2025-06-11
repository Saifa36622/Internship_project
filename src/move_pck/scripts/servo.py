#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import os
from dynamixel_sdk import * 
from std_msgs.msg import Float32MultiArray,Int32,Float32


class servo(Node):
    def __init__(self):
        super().__init__('servo')
        def setup():
            if os.name == 'nt':
                    import msvcrt
                    def getch():
                        return msvcrt.getch().decode()
            else:
                import sys, tty, termios
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                def getch():
                    try:
                        tty.setraw(sys.stdin.fileno())
                        ch = sys.stdin.read(1)
                    finally:
                        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                    return ch

            #********* DYNAMIXEL Model definition *********
            #***** (Use only one definition at a time) *****
            self.MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
            # MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
            # MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
            # MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
            # MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
            # MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V


            # Control table address
            if self.MY_DXL == 'X_SERIES' or self.MY_DXL == 'MX_SERIES':
                ADDR_TORQUE_ENABLE          = 64
                self.ADDR_GOAL_POSITION          = 116
                self.ADDR_PRESENT_POSITION       = 132
                DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
                DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
                BAUDRATE                    = 57600
            elif self.MY_DXL == 'PRO_SERIES':
                ADDR_TORQUE_ENABLE          = 562       # Control table address is different in DYNAMIXEL model
                self.ADDR_GOAL_POSITION          = 596
                self.ADDR_PRESENT_POSITION       = 611
                DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
                DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
                BAUDRATE                    = 57600
            elif self.MY_DXL == 'P_SERIES' or self.MY_DXL == 'PRO_A_SERIES':
                ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
                self.ADDR_GOAL_POSITION          = 564
                self.ADDR_PRESENT_POSITION       = 580
                DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
                DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
                BAUDRATE                    = 57600
            elif self.MY_DXL == 'XL320':
                ADDR_TORQUE_ENABLE          = 24
                self.ADDR_GOAL_POSITION          = 30
                self.ADDR_PRESENT_POSITION       = 37
                DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the CW Angle Limit of product eManual
                DXL_MAXIMUM_POSITION_VALUE  = 1023      # Refer to the CCW Angle Limit of product eManual
                BAUDRATE                    = 1000000   # Default Baudrate of XL-320 is 1Mbps

            # DYNAMIXEL Protocol Version (1.0 / 2.0)
            # https://emanual.robotis.com/docs/en/dxl/protocol2/
            PROTOCOL_VERSION            = 2.0

            # Factory default ID of all DYNAMIXEL is 1
            self.DXL_ID                      = 0

            # Use the actual port assigned to the U2D2.
            # ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
            DEVICENAME                  = '/dev/ttyUSB0'

            TORQUE_ENABLE               = 1     # Value for enabling the torque
            TORQUE_DISABLE              = 0     # Value for disabling the torque
            self.DXL_MOVING_STATUS_THRESHOLD = 30    # Dynamixel moving status threshold

            index = 0
            self.dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


            # Initialize PortHandler instance
            # Set the port path
            # Get methods and members of PortHandlerLinux or PortHandlerWindows
            self.portHandler = PortHandler(DEVICENAME)

            # Initialize PacketHandler instance
            # Set the protocol version
            # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
            self.packetHandler = PacketHandler(PROTOCOL_VERSION)

            # Open port
            if self.portHandler.openPort():
                print("Succeeded to open the port")
            else:
                print("Failed to open the port")
                print("Press any key to terminate...")
                getch()
                quit()


            # Set port baudrate
            if self.portHandler.setBaudRate(BAUDRATE):
                print("Succeeded to change the baudrate")
            else:
                print("Failed to change the baudrate")
                print("Press any key to terminate...")
                getch()
                quit()

            # Enable Dynamixel Torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel has been successfully connected")

        setup()
        self.create_timer(0.1, self.timer_callback)
        self.create_subscription(Int32, "/servo_cmd", self.servo_callback, 10)
        self.trigger = 0
        self.index = 0
        if self.index != 1 :
            if (self.MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, self.dxl_goal_position[self.index])
            else:
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, self.dxl_goal_position[self.index])
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            while self.index != 1 :
                # Read present position
                if (self.MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
                    dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
                else:
                    dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))

                print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (self.DXL_ID, self.dxl_goal_position[self.index], dxl_present_position))

                if not abs(self.dxl_goal_position[self.index] - dxl_present_position) > self.DXL_MOVING_STATUS_THRESHOLD:
                    if self.index == 0:
                        self.index = 1
                    else:
                        self.index = 0
                    self.trigger = 0
                    break
    def timer_callback(self):
        while self.trigger == 1:
            # Read present position
            if (self.MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
            else:
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (self.DXL_ID, self.dxl_goal_position[self.index], dxl_present_position))

            if not abs(self.dxl_goal_position[self.index] - dxl_present_position) > self.DXL_MOVING_STATUS_THRESHOLD:
                if self.index == 0:
                    self.index = 1
                else:
                    self.index = 0
                self.trigger = 0
                break

    def servo_callback(self,msg:Int32):
        self.trigger = 1
        if (self.MY_DXL == 'XL320'): # XL320 uses 2 byte Position Data, Check the size of data in your DYNAMIXEL's control table
            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, self.dxl_goal_position[self.index])
        else:
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_GOAL_POSITION, self.dxl_goal_position[self.index])
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
                
        

def main(args=None):
    rclpy.init(args=args)
    node = servo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
