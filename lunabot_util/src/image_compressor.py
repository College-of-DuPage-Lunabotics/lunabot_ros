#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageCompressor(Node):
    def __init__(self):
        super().__init__('image_compressor')
        
        # Declare parameters
        self.declare_parameter('jpeg_quality', 40)  # Lower = more compression
        self.declare_parameter('scale', 1.0)  # Additional downscaling if needed
        
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.scale = self.get_parameter('scale').value
        
        self.bridge = CvBridge()
        
        # Front camera compression
        self.front_sub = self.create_subscription(
            Image,
            '/camera_front/color/image_raw',
            self.front_callback,
            10)
        self.front_pub = self.create_publisher(
            CompressedImage,
            '/camera_front/color/image_compressed',
            10)
        
        # Back camera compression
        self.back_sub = self.create_subscription(
            Image,
            '/camera_back/color/image_raw',
            self.back_callback,
            10)
        self.back_pub = self.create_publisher(
            CompressedImage,
            '/camera_back/color/image_compressed',
            10)
        
        # Fisheye camera compression
        self.fisheye_sub = self.create_subscription(
            Image,
            '/camera_fisheye/color/image_raw',
            self.fisheye_callback,
            10)
        self.fisheye_pub = self.create_publisher(
            CompressedImage,
            '/camera_fisheye/color/image_compressed',
            10)
        
        self.get_logger().info(
            f'Image compressor started: quality={self.jpeg_quality}, scale={self.scale}')
    
    def compress_image(self, msg):
        """Convert and compress image with configurable quality"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Optional downscaling
            if self.scale != 1.0:
                new_width = int(cv_image.shape[1] * self.scale)
                new_height = int(cv_image.shape[0] * self.scale)
                cv_image = cv2.resize(cv_image, (new_width, new_height))
            
            # Compress with custom quality
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
            result, encoded = cv2.imencode('.jpg', cv_image, encode_param)
            
            if not result:
                self.get_logger().error('Failed to encode image')
                return None
            
            # Create CompressedImage message
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = "jpeg"
            compressed_msg.data = encoded.tobytes()
            
            return compressed_msg
            
        except Exception as e:
            self.get_logger().error(f'Compression error: {e}')
            return None
    
    def front_callback(self, msg):
        compressed = self.compress_image(msg)
        if compressed:
            self.front_pub.publish(compressed)
    
    def back_callback(self, msg):
        compressed = self.compress_image(msg)
        if compressed:
            self.back_pub.publish(compressed)
    
    def fisheye_callback(self, msg):
        compressed = self.compress_image(msg)
        if compressed:
            self.fisheye_pub.publish(compressed)


def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
