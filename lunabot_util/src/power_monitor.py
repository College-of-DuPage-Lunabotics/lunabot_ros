#!/usr/bin/env python3
import serial
import struct
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from lunabot_logger import Logger
from lunabot_msgs.msg import PowerMonitor

# Alert flag constants  
TEMPOL    = 0x80  # Temperature Over Limit
CURRENTOL = 0x40  # Current Over Limit
CURRENTUL = 0x20  # Current Under Limit
BUSOL     = 0x10  # Bus Voltage Over Limit
BUSUL     = 0x08  # Bus Voltage Under Limit
POL       = 0x04  # Power Over Limit


class PowerMonitorNode(Node):
    def __init__(self):
        super().__init__('power_monitor')
        self.log = Logger(self)

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('publish_rate', 5.0)  # Hz

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        publish_rate = self.get_parameter('publish_rate').value

        try:
            self.serial = serial.Serial(serial_port, baud_rate, timeout=1.0)
            self.log.success(f'Connected to power monitor on {serial_port} at {baud_rate} baud')
        except Exception as e:
            self.log.failure(f'Failed to open serial port {serial_port}: {e}')
            self.serial = None

        self.power_pub = self.create_publisher(PowerMonitor, '/power_monitor', 10)

        # Energy integration state
        self.total_energy_wh = 0.0
        self.last_power_w = 0.0
        self.last_time = time.time()

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        self.log.success('Power monitor initialized')
    
    def find_sync(self):
        """Find the 0xBEEF sync header"""
        if not self.serial:
            return False
            
        try:
            while True:
                byte = self.serial.read(1)
                if not byte:
                    return False
                if byte == b'\xBE':
                    byte = self.serial.read(1)
                    if byte == b'\xEF':
                        return True
        except Exception as e:
            self.log.failure(f'Error finding sync: {e}')
            return False
    
    def validate_checksum(self, data, checksum):
        """Validate XOR checksum"""
        calc_checksum = 0
        for byte in data:
            calc_checksum ^= byte
        return calc_checksum == checksum
    
    def decode_alert_flags(self, alert):
        """Decode alert flags into readable string"""
        alert_msgs = []
        if alert & TEMPOL:
            alert_msgs.append("OVER TEMP")
        if alert & CURRENTOL:
            alert_msgs.append("OVER CURRENT")
        if alert & CURRENTUL:
            alert_msgs.append("UNDER CURRENT")
        if alert & BUSOL:
            alert_msgs.append("OVER BUS VOLTAGE")
        if alert & BUSUL:
            alert_msgs.append("UNDER BUS VOLTAGE")
        if alert & POL:
            alert_msgs.append("OVER POWER")
        
        return ", ".join(alert_msgs) if alert_msgs else "OK"
    
    def integrate_energy(self, power_w):
        """Integrate power over time to calculate energy in Wh"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Average power over interval * time in hours = energy in Wh
        avg_power_w = (power_w + self.last_power_w) / 2.0
        energy_wh = avg_power_w * (dt / 3600.0)
        
        self.total_energy_wh += energy_wh
        self.last_power_w = power_w
        self.last_time = current_time
    
    def timer_callback(self):
        if not self.serial:
            return
        
        try:
            if not self.find_sync():
                return

            # 1 device_id + 4 floats + 1 uint16 + 1 checksum = 20 bytes
            data = self.serial.read(20)
            if len(data) != 20:
                self.log.warning('Incomplete packet received')
                return

            payload = data[:19]
            checksum = data[19]

            if not self.validate_checksum(payload, checksum):
                self.log.warning('Checksum error, skipping packet')
                return

            device_id, voltage, current, power, temp, alert = struct.unpack("<BffffH", payload)

            self.integrate_energy(power)
            alert_status = self.decode_alert_flags(alert)

            msg = PowerMonitor()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'power_monitor'
            
            msg.device_id = device_id
            msg.voltage = voltage
            msg.current = current
            msg.power = power
            msg.temperature = temp
            msg.energy_wh = self.total_energy_wh
            msg.alert_flags = alert
            msg.alert_status = alert_status
            
            self.power_pub.publish(msg)
            
            # Log occasionally (every 100 messages = 10 seconds at 10 Hz)
            if not hasattr(self, 'msg_count'):
                self.msg_count = 0
            self.msg_count += 1
            if self.msg_count % 100 == 0:
                self.log.info(
                    f'[Device {device_id}] V:{voltage:.2f} I:{current:.2f} '
                    f'P:{power:.2f} T:{temp:.1f} E:{self.total_energy_wh:.2f}Wh STATUS: {alert_status}'
                )
            
        except Exception as e:
            self.log.failure(f'Error reading power monitor: {e}')
    
    def destroy_node(self):
        if self.serial:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PowerMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
