#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from api import *
from std_msgs.msg import Float32MultiArray,Int32,Float32

class MIR_move(Node):
    def __init__(self):
        super().__init__('MIR_move')
        self.mir_req = MirRequestor()
        self.create_timer(0.1, self.timer_callback)
        self.create_subscription(Float32MultiArray, "/test", self.move_callback, 10)
    def timer_callback(self) :
        pass
    def move_callback(self,msg:Float32MultiArray):
        recieve_msg = msg.data
        self.mir_req.relative_move(x=recieve_msg[0], y=recieve_msg[1], orientation= 0.0)
def main(args=None):
    rclpy.init(args=args)
    node = MIR_move()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
