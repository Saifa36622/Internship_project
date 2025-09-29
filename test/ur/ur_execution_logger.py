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
        self.count = 0  
        self.gripper_state = 0  # 0 = stopped, 1 = gripping, 2 = releasing

    def start_callback(self, msg):
        self.state = 2
        
    def gripper_grip(self):
        msg = Int32()
        msg.data = 1 
        self.publisher.publish(msg)
        self.gripper_state = 1  # Set state to gripping
    
    def gripper_release(self):
        # Send release command (2)
        msg = Int32()
        msg.data = 2
        self.publisher.publish(msg)
        self.gripper_state = 2  # Set state to releasing
        
        # Wait for 3 seconds
        time.sleep(3.0)
        
        # Send stop command (0)
        msg.data = 0
        self.publisher.publish(msg)
        # self.get_logger().info('Gripper released and stopped')
        
    
    def timer_callback(self):
        # if self.state == 0:
        #     joint_q = [2.373711109161377, -2.538924833337301, -0.554995059967041, 3.0680319505878906, -1.167661492024557, 4.13826847076416]
        #     rtde_control.moveJ(joint_q)
        #     joint_q = [2.4493489265441895, -2.953338762322897, -0.8242430686950684, 3.8622066217609863, -1.4928253332721155, 3.9300923347473145]
        #     rtde_control.moveJ(joint_q)
        #     self.gripper_grip()
        #     time.sleep(3.0)
        #     joint_q = [2.49088716506958, -2.8070203266539515, -0.8166580200195312, 3.6120120722004394, -1.4364898840533655, 4.062632083892822]
        #     rtde_control.moveJ(joint_q)
        #     self.state = 1

        # elif self.state == 1:
        #     joint_q = [2.233736515045166, -3.070524831811422, -0.8195862770080566, 3.8928724962421875, -1.4917848745929163, 4.06032657623291]
        #     rtde_control.moveJ(joint_q)
        #     self.gripper_release()
        #     joint_q = [2.223121166229248, -2.544035573998922, -0.7716889381408691, 3.3404671388813476, -1.4948943297015589, 4.0583906173706055]
        #     rtde_control.moveJ(joint_q)
        #     self.state = 2
        if self.state == 2:
            # joint_q = [2.470757007598877, -2.5440474949278773, -0.7717127799987793, 3.3404194551655273, -1.494882885609762, 4.058426856994629]
            # rtde_control.moveJ(joint_q)
            joint_q = [2.457329273223877, -3.1146708927550257, -0.8202462196350098, 4.018797083491943, -1.4911988417254847, 4.045977592468262]
            rtde_control.moveJ(joint_q)
            self.gripper_grip()
            time.sleep(3.0)
            self.state = 3
        elif self.state == 3:
            joint_q = [3.1456356048583984, -3.1755501232542933, -0.6969928741455078, 3.9865667062946777, -1.3186348120318812, 4.046479225158691]
            rtde_control.moveJ(joint_q)
            self.gripper_release()
            rtde_control.moveJ(self.home_q)
            self.state = 4
        

def main(args=None):
    rclpy.init(args=args)
    node = ServoCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()