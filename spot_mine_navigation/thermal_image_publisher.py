import sys, rclpy
from rclpy.node import Node
from rclpy.impl import rcutils_logger
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import bosdyn.client
from bosdyn.client import spot_cam
from bosdyn.client.payload import PayloadClient
from spot_wrapper.cam_wrapper import ImageStreamWrapper
from spot_wrapper.wrapper import SpotWrapper

class ThermalPublisher(Node):
    def __init__(self):
        super().__init__('thermal_publisher')
        self.declare_parameter('username', "spot")
        self.declare_parameter('password', "12345")
        self.declare_parameter('hostname', "255.255.255.255")
        self.hostname = self.get_parameter('hostname').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value
        self.logger = rcutils_logger.RcutilsLogger(name=f"thermal_publisher")
        self.cv_bridge = CvBridge()

        # initialize the spotCAM payload
        port = 0
        self.sdk = bosdyn.client.create_standard_sdk("Spot CAM Client", cert_resource_glob=None)
        spot_cam.register_all_service_clients(self.sdk)
        self.robot = self.sdk.create_robot(self.hostname)
        if port: self.robot.update_secure_channel_port(port)
        SpotWrapper.authenticate(self.robot, self.username, self.password, self.logger)

        self.payload_client: PayloadClient = self.robot.ensure_client(PayloadClient.default_service_name)
        self.payload_details = None
        for payload in self.payload_client.list_payloads():
            if payload.is_enabled and "Spot CAM" in payload.name:
                self.payload_details = payload

        if not self.payload_details:
            raise SystemError(
                "Expected an enabled payload with Spot CAM in the name. This does not appear to exist. "
                "Please verify that the spot cam is correctly configured in the payload list on the "
                "admin interface"
            )

        self.image_stream = ImageStreamWrapper(self.hostname, self.robot, self.logger)
        self.last_image_time = self.image_stream.last_image_time

        self.image_pub = self.create_publisher(Image, '/spot_cam/image', 1)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        if self.last_image_time != self.image_stream.last_image_time:
            image_cv = self.image_stream.last_image[:, 190:1090]
            height, width, _ = image_cv.shape
            image_msg = self.cv_bridge.cv2_to_imgmsg(image_cv, "bgr8")
            self.image_pub.publish(image_msg)
            self.last_image_time = self.image_stream.last_image_time
            self.get_logger().debug(f"Publishing: Image-{self.i} height {height}, width {width}")
            self.i += 1

def main(args=None):
    rclpy.init(args=args)
    thermal_publisher = ThermalPublisher()
    try:
        rclpy.spin(thermal_publisher)
    except KeyboardInterrupt:
        thermal_publisher.image_stream.shutdown_flag.set()
        thermal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()