#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time
from rtde_control import RTDEControlInterface
import math
rtde_control = RTDEControlInterface("192.168.12.60")

class ServoCommandNode(Node):
    def __init__(self):
        super().__init__('servo_command_node')
        self.publisher = self.create_publisher(Int32, '/servo_cmd', 10)
        self.sub_start = self.create_subscription(Int32, '/start', self.start_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = time.time()
        self.sequence_step = 0
        self.state = 10
        self.home_q = [2.4386210441589355, -1.234729604130127, -0.7267842292785645, 1.1271859842487792, -1.7405036131488245, 3.9444682598114014]
        rtde_control.moveJ(self.home_q)
    
    def start_callback(self, msg):
        self.state = 0
        
    def gripper_grip(self):
        msg = Int32()
        msg.data = 1 
        self.publisher.publish(msg)
    
    def gripper_release(self):
        # Send release command (2)
        msg = Int32()
        msg.data = 2
        self.publisher.publish(msg)
        
        # Wait for 3 seconds
        time.sleep(3.0)
        
        # Send stop command (0)
        msg.data = 0
        self.publisher.publish(msg)
        # self.get_logger().info('Gripper released and stopped')
        
    
    def timer_callback(self):
        if self.state == 0:
            joint_q = [3.1456356048583984, -3.1755501232542933, -0.6969928741455078, 3.9865667062946777, -1.3186348120318812, 4.046479225158691]
            rtde_control.moveJ(joint_q)
            self.gripper_grip()
            time.sleep(3.0)
            self.state = 1

        elif self.state == 1:
            joint_q = [3.918604850769043, -2.092674871484274, -0.7163586616516113, 2.9081026750751953, -1.628986660634176, 4.077793121337891]
            rtde_control.moveJ(joint_q)
            self.gripper_release()
            self.state = 2
        elif self.state == 2:
            joint_q = self.home_q
            rtde_control.moveJ(joint_q)
            self.state = 10
        elif self.state == 3:
            pass
        

def main(args=None):
    rclpy.init(args=args)
    node = ServoCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()