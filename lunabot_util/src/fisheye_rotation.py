#!/usr/bin/env python3
import serial
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from lunabot_logger import Logger

CMD_SET_POSITION = 0x01


class FisheyeRotationController(Node):
    def __init__(self):
        super().__init__('fisheye_rotation')
        self.log = Logger(self)
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        
        self.ser = None
        
        # Subscribe to camera commands
        self.create_subscription(
            Float64MultiArray,
            '/camera_controller/commands',
            self.camera_command_callback,
            10
        )
        
        # Connect to serial port
        self.connect_serial()
        
        self.log.action('Fisheye rotation initialized')
    
    def connect_serial(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.5)
            self.ser.dtr = True
            self.ser.rts = True
            self.log.success(f'Connected to {self.serial_port}')
        except serial.SerialException as e:
            self.log.failure(f'Failed to connect to {self.serial_port}: {e}')
            self.ser = None
    
    def radians_to_degrees(self, radians):
        return int(max(0, min(300, math.degrees(radians))))

    def degrees_to_position(self, degrees):
        return int((degrees * 1024) / 300)

    def camera_command_callback(self, msg):
        if len(msg.data) < 1:
            return
        
        position_radians = msg.data[0]
        target_degrees = self.radians_to_degrees(position_radians)
        self.set_position(target_degrees)
    
    def set_position(self, degrees):
        if not self.ser or not self.ser.is_open:
            return
        
        try:
            position = self.degrees_to_position(degrees)
            packet = bytes([
                CMD_SET_POSITION,
                (position >> 8) & 0xFF,
                position & 0xFF
            ])
            self.ser.write(packet)
            response = self.ser.read(1)
            if len(response) == 1 and response[0] == 0:
                self.log.action(f'Rotated to {degrees}°')
                
        except serial.SerialException as e:
            self.log.failure(f'Serial error: {e}')
            self.ser = None
    
    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FisheyeRotationController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
