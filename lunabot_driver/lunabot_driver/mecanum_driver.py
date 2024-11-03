#!/usr/bin/env python
# encoding: utf-8

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from Rosmaster_Lib import Rosmaster


class MecanumDriver(Node):
    def __init__(self, name):
        super().__init__(name)
        self.car = Rosmaster()
        self.car.set_car_type(1)

        # Declare and load scaling parameters for the robot's speed
        self.declare_parameter("linear_scale", 100.0)
        self.declare_parameter("angular_scale", 100.0)
        self.linear_scale = self.get_parameter("linear_scale").value
        self.angular_scale = self.get_parameter("angular_scale").value

        # Subscribe to the joystick topic
        self.sub_joy = self.create_subscription(Joy, "joy", self.joy_callback, 1)

        # Initialize joystick control variables
        self.joystick_linear_x = 0.0
        self.joystick_linear_y = 0.0
        self.joystick_angular_z = 0.0

    def joy_callback(self, msg):
        # Map joystick inputs to movement commands
        self.joystick_linear_x = msg.axes[4] * self.linear_scale  # Right joystick up/down
        self.joystick_linear_y = msg.axes[0] * self.linear_scale  # Left joystick left/right
        self.joystick_angular_z = msg.axes[3] * self.angular_scale  # Right joystick left/right

        # Calculate and send wheel speeds based on joystick input
        self.update_wheel_speeds()

    def update_wheel_speeds(self):
        # Calculate the desired magnitude and angle of translation
        vx = self.joystick_linear_x
        vy = self.joystick_linear_y
        v_theta = self.joystick_angular_z

        # Calculate wheel speeds using the holonomic drive formula
        v_d = math.sqrt(vx ** 2 + vy ** 2)
        theta_d = math.atan2(vy, vx)

        front_left = v_d * math.sin(theta_d + math.pi / 4) + v_theta
        front_right = v_d * math.cos(theta_d + math.pi / 4) - v_theta
        back_left = v_d * math.cos(theta_d + math.pi / 4) + v_theta
        back_right = v_d * math.sin(theta_d + math.pi / 4) - v_theta

        # Clamp wheel speeds to range [-100, 100]
        front_left = max(min(front_left, 100), -100)
        front_right = max(min(front_right, 100), -100)
        back_left = max(min(back_left, 100), -100)
        back_right = max(min(back_right, 100), -100)
        
        # Send clamped speeds to motors
        self.car.set_motor(int(front_left), int(front_right), int(back_left), int(back_right))


def main():
    rclpy.init()
    driver = MecanumDriver("mecanum_driver")
    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()
