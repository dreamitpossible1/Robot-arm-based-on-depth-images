#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import sys

class SingleShot(Node):
    def __init__(self):
        super().__init__('single_shot')
        # Subscribe with QoS=1 (best effort, single message)
        self.sub = self.create_subscription(
            Image,
            '/image',
            self.callback,
            qos_profile=1  # Critical for single capture
        )
        self.bridge = CvBridge()
        self.captured = False

    def callback(self, msg):
        if not self.captured:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                os.makedirs('/home/root/saved_images', exist_ok=True)
                cv2.imwrite('/home/root/saved_images/image.jpg', cv_image)
                self.get_logger().info("Saved image.jpg - Exiting now")
                self.captured = True
                # Forceful exit
                self.destroy_node()
                rclpy.try_shutdown()
                sys.exit(0)
            except Exception as e:
                self.get_logger().error(f"Failed: {e}")
                sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    node = SingleShot()
    # Single-threaded spin with timeout
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        # Timeout after 5 sec if no image arrives
        executor.spin_once(timeout_sec=5.0)
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()