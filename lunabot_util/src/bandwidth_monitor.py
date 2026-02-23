import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import time
import os


class BandwidthMonitor(Node):
    def __init__(self):
        super().__init__('bandwidth_monitor')
        
        # Declare parameters
        self.declare_parameter('interface', 'auto')  # 'auto' or specific like 'eth0'
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('warning_threshold', 8.0)  # Mbps
        self.declare_parameter('error_threshold', 10.0)  # Mbps
        
        # Get parameters
        interface = self.get_parameter('interface').value
        update_rate = self.get_parameter('update_rate').value
        self.warning_threshold = self.get_parameter('warning_threshold').value
        self.error_threshold = self.get_parameter('error_threshold').value
        
        # Detect interface
        self.interface = self.detect_interface() if interface == 'auto' else interface
        
        if not self.interface:
            self.get_logger().error('No ethernet interface found!')
            return
        
        self.get_logger().info(f'Monitoring interface: {self.interface}')
        
        # Publishers
        self.rx_pub = self.create_publisher(Float32, 'bandwidth/rx_mbps', 10)
        self.tx_pub = self.create_publisher(Float32, 'bandwidth/tx_mbps', 10)
        self.total_pub = self.create_publisher(Float32, 'bandwidth/total_mbps', 10)
        self.status_pub = self.create_publisher(String, 'bandwidth/status', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        
        # State
        self.last_rx_bytes = self.get_bytes('rx')
        self.last_tx_bytes = self.get_bytes('tx')
        self.last_time = time.time()
        
        # Timer
        self.timer = self.create_timer(1.0 / update_rate, self.update)
        
        self.get_logger().info('Bandwidth monitor started')
    
    def detect_interface(self):
        """Auto-detect ethernet interface."""
        try:
            interfaces = os.listdir('/sys/class/net/')
            for iface in interfaces:
                if iface.startswith(('eth', 'enp', 'eno')):
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
        
        # Determine status
        if total_mbps > self.error_threshold:
            status = f'ERROR: {total_mbps:.2f} Mbps (limit: {self.error_threshold})'
            level = DiagnosticStatus.ERROR
        elif total_mbps > self.warning_threshold:
            status = f'WARNING: {total_mbps:.2f} Mbps'
            level = DiagnosticStatus.WARN
        else:
            status = f'OK: {total_mbps:.2f} Mbps'
            level = DiagnosticStatus.OK
        
        self.status_pub.publish(String(data=status))
        
        # Publish diagnostics
        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        
        diag_status = DiagnosticStatus()
        diag_status.name = 'Bandwidth Monitor'
        diag_status.level = level
        diag_status.message = status
        diag_status.hardware_id = self.interface
        
        diag_status.values = [
            KeyValue(key='RX (Mbps)', value=f'{rx_mbps:.2f}'),
            KeyValue(key='TX (Mbps)', value=f'{tx_mbps:.2f}'),
            KeyValue(key='Total (Mbps)', value=f'{total_mbps:.2f}'),
            KeyValue(key='Warning Threshold', value=f'{self.warning_threshold:.1f}'),
            KeyValue(key='Error Threshold', value=f'{self.error_threshold:.1f}')
        ]
        
        diag_msg.status.append(diag_status)
        self.diag_pub.publish(diag_msg)
        
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
