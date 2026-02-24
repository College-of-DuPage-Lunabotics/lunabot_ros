#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import os


class BandwidthMonitor(Node):
    def __init__(self):
        super().__init__('bandwidth_monitor')
        
        # Declare parameters
        self.declare_parameter('interface', 'auto')  # 'auto' or specific like 'eth0'
        self.declare_parameter('update_rate', 1.0)  # Hz
        
        # Get parameters
        interface = self.get_parameter('interface').value
        update_rate = self.get_parameter('update_rate').value
        
        # Detect interface
        self.interface = self.detect_interface() if interface == 'auto' else interface
        
        if not self.interface:
            self.get_logger().error('No network interface found!')
            return
        
        self.get_logger().info(f'Monitoring interface: {self.interface}')
        
        # Publishers
        self.rx_pub = self.create_publisher(Float32, 'bandwidth/rx_mbps', 10)
        self.tx_pub = self.create_publisher(Float32, 'bandwidth/tx_mbps', 10)
        self.total_pub = self.create_publisher(Float32, 'bandwidth/total_mbps', 10)
        
        # State
        self.last_rx_bytes = self.get_bytes('rx')
        self.last_tx_bytes = self.get_bytes('tx')
        self.last_time = time.time()
        
        # Timer
        self.timer = self.create_timer(1.0 / update_rate, self.update)
        
        self.get_logger().info('Bandwidth monitor started')
    
    def detect_interface(self):
        """Auto-detect network interface (ethernet or WiFi)."""
        try:
            interfaces = os.listdir('/sys/class/net/')
            # Try ethernet first
            for iface in interfaces:
                if iface.startswith(('eth', 'enp', 'eno')):
                    return iface
            # If no ethernet, try WiFi
            for iface in interfaces:
                if iface.startswith(('wlan', 'wlp', 'wlo')):
                    return iface
        except Exception as e:
            self.get_logger().error(f'Failed to detect interface: {e}')
        return None
    
    def get_bytes(self, direction):
        """Get current byte count for rx or tx."""
        try:
            path = f'/sys/class/net/{self.interface}/statistics/{direction}_bytes'
            with open(path, 'r') as f:
                return int(f.read().strip())
        except Exception as e:
            self.get_logger().error(f'Failed to read {direction} bytes: {e}')
            return 0
    
    def update(self):
        """Calculate and publish bandwidth."""
        # Get current values
        current_rx = self.get_bytes('rx')
        current_tx = self.get_bytes('tx')
        current_time = time.time()
        
        # Calculate deltas
        delta_time = current_time - self.last_time
        delta_rx = current_rx - self.last_rx_bytes
        delta_tx = current_tx - self.last_tx_bytes
        
        # Calculate Mbps
        rx_mbps = (delta_rx * 8) / (delta_time * 1_000_000)
        tx_mbps = (delta_tx * 8) / (delta_time * 1_000_000)
        total_mbps = rx_mbps + tx_mbps
        
        # Publish
        self.rx_pub.publish(Float32(data=rx_mbps))
        self.tx_pub.publish(Float32(data=tx_mbps))
        self.total_pub.publish(Float32(data=total_mbps))
        
        # Update state
        self.last_rx_bytes = current_rx
        self.last_tx_bytes = current_tx
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = BandwidthMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
