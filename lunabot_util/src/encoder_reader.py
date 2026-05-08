#!/usr/bin/env python3
import struct
import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from lunabot_logger import Logger

ENCODER_CAN_ID = 0x100
CAN_FRAME_FORMAT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)


class EncoderReader(Node):
    def __init__(self):
        super().__init__('encoder_reader')
        self.log = Logger(self)

        self.declare_parameter('can_interface', 'can0')
        
        self.can_interface = self.get_parameter('can_interface').value
        self.home_offset = 3.1  # Calibrated home position offset

        self.actuator_position_pub = self.create_publisher(Float64, 'actuator_position', 10)
        self.bucket_angle_pub = self.create_publisher(Float64, 'bucket_angle', 10)

        if not self.init_can():
            self.log.failure('Failed to initialize CAN interface')
            raise RuntimeError('CAN initialization failed')

        self.timer = self.create_timer(0.005, self.update_loop)
        self.log.success(f'Encoder reader initialized on {self.can_interface} (home offset: {self.home_offset:.2f} rad)')

    def init_can(self):
        try:
            self.can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.can_socket.setblocking(False)
            self.can_socket.bind((self.can_interface,))
            return True
        except Exception as e:
            self.log.failure(f'Failed to create CAN socket: {e}')
            return False

    def receive_can_frame(self):
        try:
            frame = self.can_socket.recv(CAN_FRAME_SIZE)
            can_id, dlc, data = struct.unpack(CAN_FRAME_FORMAT, frame)
            return can_id, data[:dlc]
        except BlockingIOError:
            return None, None
        except Exception as e:
            self.log.warning(f'Error receiving CAN frame: {e}')
            return None, None

    def update_loop(self):
        while True:
            can_id, data = self.receive_can_frame()
            
            if can_id is None:
                break
            
            if can_id == ENCODER_CAN_ID and data and len(data) >= 4:
                try:
                    raw_position = struct.unpack('<f', data[:4])[0]
                    corrected_position = raw_position - self.home_offset
                    
                    msg = Float64()
                    msg.data = corrected_position
                    self.actuator_position_pub.publish(msg)
                    self.bucket_angle_pub.publish(msg)
                except Exception as e:
                    self.log.warning(f'Error decoding position: {e}')

    def destroy_node(self):
        if hasattr(self, 'can_socket') and self.can_socket:
            self.can_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = EncoderReader()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Exception: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
