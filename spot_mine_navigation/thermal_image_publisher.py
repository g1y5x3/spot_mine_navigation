import rclpy
from rclpy.node import Node
from rclpy.impl import rcutils_logger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import bosdyn.client
from bosdyn.client import spot_cam
from bosdyn.client.payload import PayloadClient
from bosdyn.client.spot_cam.ptz import PtzClient
from bosdyn.client.spot_cam.compositor import CompositorClient
from bosdyn.client.spot_cam.streamquality import StreamQualityClient
from bosdyn.api.spot_cam import ptz_pb2, streamquality_pb2

from spot_wrapper.cam_wrapper import ImageStreamWrapper
from spot_wrapper.wrapper import SpotWrapper

class ThermalPublisher(Node):
    def __init__(self):
        super().__init__('thermal_publisher')
        # TODO pass the parameters through launch
        self.declare_parameter('username', "spot")
        self.declare_parameter('password', "12345")
        self.declare_parameter('hostname', "255.255.255.255")
        self.hostname = self.get_parameter('hostname').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value
        self.logger = rcutils_logger.RcutilsLogger(name=f"thermal_publisher")
        self.cv_bridge = CvBridge()

        # initialize the spotCAM payload, mostly just from the cam_wrapper
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

        # set to the default ptz position
        ptz_desc = ptz_pb2.PtzDescription(name="mech")
        ptz_position = self.robot.ensure_client(PtzClient.default_service_name).set_ptz_position(ptz_desc, 325, 0, 1)
        self.get_logger().info(f"PTZ Position: pan {ptz_position.pan.value}, tilt {ptz_position.tilt.value}, zoom {ptz_position.zoom.value}")

        # set the thermal composition
        comp_result = self.robot.ensure_client(CompositorClient.default_service_name).set_screen("mech_full_ir")
        self.get_logger().info(f"Mode: {comp_result}")

        # set to the lower bitrate for streaming
        stream_result = self.robot.ensure_client(StreamQualityClient.default_service_name).set_stream_params(
            1000000, None, None, 1, streamquality_pb2.StreamParams.AutoExposure(), None, None)
        self.get_logger().info(f"Target Bitrate: {stream_result.targetbitrate}")

        self.image_stream = ImageStreamWrapper(self.hostname, self.robot, self.logger)
        self.last_image_time = self.image_stream.last_image_time

        self.image_pub = self.create_publisher(Image, '/camera/spot_cam', 1)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # converted from ROS 1 https://github.com/heuristicus/spot_ros/blob/master/spot_cam/src/spot_cam/spot_cam_ros.py#L1179
        # 190:1090 is used to crop out the colormap
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