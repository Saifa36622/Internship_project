#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
from rtde_control import RTDEControlInterface

class URTrackerNode(Node):
    def __init__(self):
        super().__init__('ur_tracker')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/oak_d_owl/detections',
            self.detection_callback,
            10)
        self.rtde_control = RTDEControlInterface("192.168.12.60")
        self.joint_q = [math.radians(45), -1.57, 0.0, -1.57, math.radians(90), math.radians(45)]
        self.image_width = 1280  # Should match OAK-D image width
        self.center_threshold = 150 # Pixels tolerance for 'centered'
        self.last_action_time = self.get_clock().now()
        self.action_interval = 0.01  # seconds between actions

    def detection_callback(self, msg):
        # msg.data: [label_id, score, x1, y1, x2, y2, ...] for each detection
        if not msg.data:
            return  # No detection
        # Take the first detection
        label_id, score, x1, y1, x2, y2 = msg.data[:6]
        # Compute center of bounding box
        obj_center_x = (x1 + x2) / 2.0
        image_center_x = self.image_width / 2.0
        offset = obj_center_x - image_center_x
        self.get_logger().info(f"Detection offset: {offset}")
        # Only act if enough time has passed
        now = self.get_clock().now()
        if (now - self.last_action_time).nanoseconds / 1e9 < self.action_interval:
            return
        self.last_action_time = now
        # If object is not centered, rotate joint 0
        if abs(offset) > self.center_threshold:
            # Simple proportional control: rotate joint 0 left/right
            angle_step = math.radians(5)  # Adjust as needed
            if offset < 0:
                self.joint_q[0] += angle_step  # Object is left, rotate left
            else:
                self.joint_q[0] -= angle_step  # Object is right, rotate right
            self.get_logger().info(f"Moving joint 0 to {math.degrees(self.joint_q[0])} degrees")
            self.rtde_control.moveJ(self.joint_q)

    def destroy_node(self):
        self.rtde_control.stopScript()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = URTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
