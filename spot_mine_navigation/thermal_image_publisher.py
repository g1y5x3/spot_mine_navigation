import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from spot_wrapper.cam_wrapper import SpotCamCamera, SpotCamWrapper

class thermalPublisher(Node):
    def __init__(self):
        super().__init__('thermal_publisher')
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
    thermal_publisher = thermalPublisher()
    rclpy.spin(thermal_publisher)

    thermal_publisher.destroy_node()
    rclpy.shutdown()
 

if __name__ == "__main__":
    main()