#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from livox_ros_driver2.msg import CustomMsg


class LivoxToPointCloud(Node):
    def __init__(self):
        super().__init__('livox_to_pointcloud')
        
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/pointcloud')
        self.declare_parameter('output_frame', 'livox_frame')
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('max_range', 50.0)
        
        self.output_frame = self.get_parameter('output_frame').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        
        self.pub = self.create_publisher(PointCloud2, self.get_parameter('output_topic').value, 10)
        self.sub = self.create_subscription(CustomMsg, self.get_parameter('input_topic').value, self.callback, 10)
        
    def callback(self, msg):
        if not msg.points or msg.point_num == 0:
            return
        
        valid_points = [
            p for p in msg.points
            if np.isfinite([p.x, p.y, p.z]).all() and
            self.min_range <= np.sqrt(p.x**2 + p.y**2 + p.z**2) <= self.max_range
        ]
        
        if not valid_points:
            return
            
        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = self.output_frame
        cloud.height = 1
        cloud.width = len(valid_points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = 16 * len(valid_points)
        cloud.is_dense = False
        
        points_array = np.array([[p.x, p.y, p.z, float(p.reflectivity)] for p in valid_points], dtype=np.float32)
        cloud.data = points_array.tobytes()
        
        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = LivoxToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

