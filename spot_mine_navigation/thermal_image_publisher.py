import rclpy
from rclpy.node import Node
from rclpy.impl import rcutils_logger
from std_msgs.msg import String

from spot_wrapper.cam_webrtc_client import WebRTCClient
from spot_wrapper.wrapper import SpotWrapper

class thermalPublisher(Node):
    def __init__(self):
        super().__init__('thermal_publisher')
        self.declare_parameter('username', "spot")
        self.declare_parameter('password', "12345")
        self.declare_parameter('hostname', "255.255.255.255")
        self.hostname = self.get_parameter('hostname').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value

        print(self.hostname)
        print(self.username)
        print(self.password)

        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello World: {self.i}"  
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: \"{msg.data}\"")
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    logger = rcutils_logger.RcutilsLogger(name=f"thermal_publisher")
    thermal_publisher = thermalPublisher()
    rclpy.spin(thermal_publisher)

    thermal_publisher.destroy_node()
    rclpy.shutdown()
 

if __name__ == "__main__":
    main()