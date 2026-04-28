#!/usr/bin/env python3
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from lunabot_logger import Logger


class BandwidthMonitor(Node):
    def __init__(self):
        super().__init__("bandwidth_monitor")
        self.log = Logger(self)

        self.declare_parameter("interface", "auto")
        self.declare_parameter("update_rate", 0.5)

        interface = self.get_parameter("interface").value
        update_rate = self.get_parameter("update_rate").value

        self.interface = self.detect_interface() if interface == "auto" else interface

        if not self.interface:
            self.log.failure("No network interface found!")
            return

        self.log.info(f"Monitoring interface: {self.interface}")

        self.rx_pub       = self.create_publisher(Float32, "bandwidth/rx_mbps",       10)
        self.tx_pub       = self.create_publisher(Float32, "bandwidth/tx_mbps",       10)
        self.total_pub    = self.create_publisher(Float32, "bandwidth/total_mbps",    10)
        self.avg_rx_pub   = self.create_publisher(Float32, "bandwidth/avg_rx_mbps",   10)
        self.avg_tx_pub   = self.create_publisher(Float32, "bandwidth/avg_tx_mbps",   10)
        self.avg_total_pub = self.create_publisher(Float32, "bandwidth/avg_total_mbps", 10)

        self.last_rx_bytes = self.get_bytes("rx")
        self.last_tx_bytes = self.get_bytes("tx")
        self.last_time = time.time()
        self.start_rx_bytes = self.last_rx_bytes
        self.start_tx_bytes = self.last_tx_bytes
        self.start_time = self.last_time

        self.timer = self.create_timer(1.0 / update_rate, self.update)

        self.log.success("Bandwidth monitor started")

    def detect_interface(self):
        try:
            interfaces = os.listdir("/sys/class/net/")
            # Try ethernet first
            for iface in interfaces:
                if iface.startswith(("eth", "enp", "eno")):
                    return iface
            # If no ethernet, try WiFi
            for iface in interfaces:
                if iface.startswith(("wlan", "wlp", "wlo")):
                    return iface
        except Exception as e:
            self.log.failure(f"Failed to detect interface: {e}")
        return None

    def get_bytes(self, direction):
        try:
            path = f"/sys/class/net/{self.interface}/statistics/{direction}_bytes"
            with open(path, "r") as f:
                return int(f.read().strip())
        except Exception as e:
            self.log.failure(f"Failed to read {direction} bytes: {e}")
            return 0

    def update(self):
        current_rx = self.get_bytes("rx")
        current_tx = self.get_bytes("tx")
        current_time = time.time()

        delta_time = current_time - self.last_time
        delta_rx = current_rx - self.last_rx_bytes
        delta_tx = current_tx - self.last_tx_bytes

        rx_mbps    = (delta_rx * 8) / (delta_time * 1_000_000)
        tx_mbps    = (delta_tx * 8) / (delta_time * 1_000_000)
        total_mbps = rx_mbps + tx_mbps

        total_time = current_time - self.start_time
        if total_time > 0:
            total_rx_bytes = current_rx - self.start_rx_bytes
            total_tx_bytes = current_tx - self.start_tx_bytes

            avg_rx_mbps = (total_rx_bytes * 8) / (total_time * 1_000_000)
            avg_tx_mbps = (total_tx_bytes * 8) / (total_time * 1_000_000)
            avg_total_mbps = avg_rx_mbps + avg_tx_mbps
        else:
            avg_rx_mbps = 0.0
            avg_tx_mbps = 0.0
            avg_total_mbps = 0.0

        self.rx_pub.publish(Float32(data=rx_mbps))
        self.tx_pub.publish(Float32(data=tx_mbps))
        self.total_pub.publish(Float32(data=total_mbps))

        self.avg_rx_pub.publish(Float32(data=avg_rx_mbps))
        self.avg_tx_pub.publish(Float32(data=avg_tx_mbps))
        self.avg_total_pub.publish(Float32(data=avg_total_mbps))

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


if __name__ == "__main__":
    main()
