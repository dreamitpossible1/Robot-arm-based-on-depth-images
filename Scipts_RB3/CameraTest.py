#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.image_count = 0
        os.makedirs('/home/root/saved_images', exist_ok=True)

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            filename = f"/home/root/saved_images/image_{self.image_count:04d}.jpg"
            cv2.imwrite(filename, cv_image)
            self.get_logger().info(f"Saved {filename}")
            self.image_count += 1
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    print("Image saver node started")
    image_saver = ImageSaver()
    print("Image saver node initialized")
    rclpy.spin(image_saver)
    print("Image saver node stopped")
    image_saver.destroy_node()
    print("Image saver node destroyed")
    rclpy.shutdown()

if __name__ == '__main__':
    print("Starting image saver node...")
    main()
    

    