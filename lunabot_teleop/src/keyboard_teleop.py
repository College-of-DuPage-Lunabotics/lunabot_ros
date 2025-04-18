#!/usr/bin/env python3

import sys
import threading
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

HELP_MSG = """
---------------------------
Drive around with WASD:
        w
   a    s    d

Max speed: 0.75
Adjust speed:
q : Increase linear speed
z : Decrease linear speed
e : Increase angular speed
c : Decrease angular speed

Lift the blade with Up/Down arrows:
Up Arrow: Increase position
Down Arrow: Decrease position

Tilt the blade with Left/Right arrows:
Left Arrow: Counter-clockwise
Right Arrow: Clockwise

CTRL-C to quit
"""

MOVE_BINDINGS = {
    "w": (1, 0, 0, 0),
    "s": (-1, 0, 0, 0),
    "a": (0, 0, 0, 1),
    "d": (0, 0, 0, -1),
}

SPEED_BINDINGS = {
    "q": (1.1, 1.0),
    "e": (1.0, 1.1),
    "z": (0.9, 1.0),
    "c": (1.0, 0.9),
}

BLADE_BINDINGS = {
    "\x1b[A": -0.025,
    "\x1b[B": 0.025,
    "\x1b[D": -0.025,
    "\x1b[C": 0.025,
}


def save_terminal_settings():
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def get_key(old_settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == "\x1b":
        key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key


def format_velocity(linear_speed, angular_speed):
    return f"Current:\tlinear {linear_speed}\tangular {angular_speed}"


def main():
    old_settings = save_terminal_settings()
    rclpy.init()

    node = rclpy.create_node("keyboard_teleop")
    cmd_vel_pub = node.create_publisher(Twist, "/cmd_vel", 10)
    blade_pub = node.create_publisher(Float64MultiArray, "/position_controller/commands", 10)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()

    speed = 0.35
    turn = 0.25
    max_speed = 0.75
    max_turn = 0.75
    blade_pos = 0.0
    blade_arm_pos = 0.0
    max_blade_pos = 0.75
    min_blade_pos = -0.75
    max_blade_arm_pos = 1.0
    min_blade_arm_pos = -1.0
    x = 0.0
    theta = 0.0
    status = 0

    try:
        print(HELP_MSG)
        print(format_velocity(speed, turn))

        while True:
            key = get_key(old_settings)

            if key in MOVE_BINDINGS:
                x = MOVE_BINDINGS[key][0]
                theta = MOVE_BINDINGS[key][3]
                twist = Twist()
                twist.linear.x = x * speed
                twist.angular.z = theta * turn
                cmd_vel_pub.publish(twist)

            elif key in SPEED_BINDINGS:
                speed *= SPEED_BINDINGS[key][0]
                turn *= SPEED_BINDINGS[key][1]
                speed = min(max_speed, max(0.0, speed))
                turn = min(max_turn, max(0.0, turn))
                print(format_velocity(speed, turn))
                if status == 14:
                    print(HELP_MSG)
                status = (status + 1) % 15

            elif key in BLADE_BINDINGS:
                if key in ("\x1b[A", "\x1b[B"):
                    blade_arm_pos = max(
                        min(blade_arm_pos + BLADE_BINDINGS[key], max_blade_arm_pos),
                        min_blade_arm_pos,
                    )
                else:  # left/right arrows
                    blade_pos = max(
                        min(blade_pos + BLADE_BINDINGS[key], max_blade_pos), min_blade_pos
                    )
                blade_msg = Float64MultiArray(data=[blade_pos, blade_arm_pos])
                blade_pub.publish(blade_msg)

            else:
                x = 0.0
                theta = 0.0
                if key == "\x03":
                    break

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)

        restore_terminal_settings(old_settings)
        rclpy.shutdown()
        spin_thread.join()


if __name__ == "__main__":
    main()
