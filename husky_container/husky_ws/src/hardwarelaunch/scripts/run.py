#!/usr/bin/env python3

import time
from datetime import datetime

import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class RealsenseGrabber(Node):
    def __init__(self):
        super().__init__('realsense_grabber')
        self.bridge = CvBridge()

        # Subscribers for the two camera topics
        self.create_subscription(
            Image, 'Cam1', self.callback1, 10)
        self.create_subscription(
            Image, 'Cam2', self.callback2, 10)

    def callback1(self, msg: Image):
        self._handle_frame(msg, window_name='Frame1', prefix='D445_Frame1_')

    def callback2(self, msg: Image):
        self._handle_frame(msg, window_name='Frame2', prefix='D445_Frame2_')

    def _handle_frame(self, msg, window_name: str, prefix: str):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            self.get_logger().info(f'Read image for {window_name}')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        now = datetime.now()
        timestamp = now.strftime('%H-%M')
        img_name = f'{prefix}{timestamp}.jpg'

        cv.imshow(window_name, frame)
        # Change path here if you want to save elsewhere
        cv.imwrite(img_name, frame)

        # block for 15 minutes
        time.sleep(900)


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseGrabber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
