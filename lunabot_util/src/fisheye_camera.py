#!/usr/bin/env python3
import threading

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from lunabot_logger import Logger


class FisheyeCameraDriver(Node):
    def __init__(self):
        super().__init__('fisheye_camera')
        self.log = Logger(self)

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
        self._running = False
        self._thread = None

        # Depth 1: only the latest frame matters
        self.compressed_pub = self.create_publisher(
            CompressedImage, '/camera_fisheye/color/image_compressed', 1)

        if self.open_camera():
            self._running = True
            self._thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._thread.start()
            self.log.action('Fisheye camera initialized')

    def open_camera(self):
        try:
            device = int(self.device_id) if self.device_id.isdigit() else self.device_id

            # Use GStreamer to request MJPEG natively from V4L2, avoiding YUYV 15fps limit
            gst_pipeline = (
                f'v4l2src device={self.device_id} ! '
                f'image/jpeg,width={self.width},height={self.height},framerate={self.fps}/1 ! '
                f'jpegdec ! videoconvert ! appsink drop=true max-buffers=1'
            )
            self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)

            if not self.cap.isOpened():
                self.log.warning('GStreamer pipeline failed, falling back to default capture')
                self.cap = cv2.VideoCapture(device)
                if not self.cap.isOpened():
                    self.log.failure(f'Failed to open camera: {self.device_id}')
                    return False
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

            actual_w   = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h   = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))

            self.log.success(
                f'Opened {self.device_id}: {actual_w}x{actual_h}@{actual_fps}fps, '
                f'quality={self.jpeg_quality}')
            return True
        except Exception as e:
            self.log.failure(f'Error opening camera: {e}')
            return False

    def _capture_loop(self):
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        while self._running and self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret or frame is None:
                continue
            result, encoded = cv2.imencode('.jpg', frame, encode_params)
            if result:
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.format = 'jpeg'
                msg.data = encoded.tobytes()
                self.compressed_pub.publish(msg)

    def destroy_node(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
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
