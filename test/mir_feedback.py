# import rclpy
# from rclpy.node import Node
# import requests
# from requests.auth import HTTPBasicAuth
# from std_msgs.msg import String


# class MirAPIClient(Node):
#     def __init__(self):
#         super().__init__('mir_api_client')
#         self.base_url = "http://192.168.12.20/api/v2.0.0/"
#         self.auth = HTTPBasicAuth('distributor', 'distributor')

#         # Timer: Call get_status every 5 seconds
#         self.timer = self.create_timer(0.1, self.get_status)

#         # Publisher example (you can publish status)
#         self.status_publisher = self.create_publisher(String, 'mir_status', 10)

#     def get_status(self):
#         try:
#             url = self.base_url + "status"
#             response = requests.get(url, auth=self.auth, timeout=2)

#             if response.status_code == 200:
#                 status = response.json()
#                 self.get_logger().info(f"Status: {status}")
                
#                 # Publish status as JSON string
#                 msg = String()
#                 msg.data = str(status)
#                 self.status_publisher.publish(msg)

#             else:
#                 self.get_logger().warn(f"HTTP {response.status_code}: {response.text}")

#         except requests.RequestException as e:
#             self.get_logger().error(f"Request failed: {e}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = MirAPIClient()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
import requests
from requests.auth import HTTPBasicAuth
from std_msgs.msg import String


class MirAPIClient(Node):
    def __init__(self):
        super().__init__('mir_api_client')
        self.base_url = "http://192.168.12.20/api/v2.0.0/"
        self.auth = HTTPBasicAuth('distributor', 'distributor')

        # Timer: Call get_status every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.1, self.get_status)

        # Publisher (can still use full status if needed elsewhere)
        self.status_publisher = self.create_publisher(String, 'mir_status', 10)

    def get_status(self):
        try:
            url = self.base_url + "status"
            response = requests.get(url, auth=self.auth, timeout=2)

            if response.status_code == 200:
                status = response.json()

                # Extract essential information
                position = status.get("position", {})
                velocity = status.get("velocity", {})
                battery = status.get("battery_percentage", None)
                state = status.get("state_text", "")
                mission_text = status.get("mission_text", "")

                # Log essential information
                self.get_logger().info(
                    f"[STATE: {state}] "
                    f"Position -> x: {position.get('x'):.2f}, y: {position.get('y'):.2f}, orientation: {position.get('orientation'):.2f} | "
                    f"Velocity -> linear: {velocity.get('linear'):.2f}, angular: {velocity.get('angular'):.2f} | "
                    f"Battery: {battery:.2f}% | "
                    f"Mission: {mission_text}"
                )

                # Optionally publish full JSON string
                msg = String()
                msg.data = str(status)
                self.status_publisher.publish(msg)

            else:
                self.get_logger().warn(f"HTTP {response.status_code}: {response.text}")

        except requests.RequestException as e:
            self.get_logger().error(f"Request failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MirAPIClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
