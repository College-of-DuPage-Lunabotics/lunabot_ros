#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

from lunabot_logger import Logger


class FisheyeCameraDriver(Node):
    def __init__(self):
        super().__init__('fisheye_camera')
        self.log = Logger(self)
        
        # Parameters
        self.declare_parameter('device_id', '/dev/video0')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 80)
        
        self.device_id = self.get_parameter('device_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        self.cap = None
        
        # Publisher - only compressed image for GUI
        self.compressed_pub = self.create_publisher(
            CompressedImage, 
            '/camera_fisheye/color/image_compressed', 
            10
        )
        
        # Open camera
        self.open_camera()
        
        # Setup capture timer
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.capture_and_publish)
        
        self.log.action('Fisheye camera initialized')
    
    def open_camera(self):
        """Open the USB camera device"""
        try:
            device = int(self.device_id) if self.device_id.isdigit() else self.device_id
            self.cap = cv2.VideoCapture(device)
            
            if not self.cap.isOpened():
                self.log.failure(f'Failed to open camera: {self.device_id}')
                return False
            
            # Set camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            self.log.success(f'Opened camera {self.device_id}')
            return True
            
        except Exception as e:
            self.log.failure(f'Error opening camera: {e}')
            return False
    
    def capture_and_publish(self):
        """Capture frame and publish compressed image"""
        if not self.cap or not self.cap.isOpened():
            return
        
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return
        
        # Encode as JPEG and publish
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        result, encoded_img = cv2.imencode('.jpg', frame, encode_param)
        
        if result:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = 'jpeg'
            msg.data = np.array(encoded_img).tobytes()
            self.compressed_pub.publish(msg)
    
    def destroy_node(self):
        """Clean up on shutdown"""
        if self.cap and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FisheyeCameraDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
