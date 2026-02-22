/**
 * @file controller_teleop.cpp
 * @author Grayson Arendt
 * @date 4/17/2025
 */

#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "SparkMax.hpp"

#define WHEEL_RADIUS 0.095  // In meters
#define WHEEL_BASE 0.53

// ANSI color codes
#define RESET "\033[0m"
#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define MAGENTA "\033[1;35m"

/**
 * @class ControllerTeleop
 * @brief Handles joystick/navigation commands and drives motors.
 */
class ControllerTeleop : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for ControllerTeleop.
   */
  ControllerTeleop()
    : Node("controller_teleop")
    , right_actuator_motor_("can0", 1)
    , left_actuator_motor_("can0", 2)
    , right_wheel_motor_("can0", 3)
    , left_wheel_motor_("can0", 4)
    , vibration_motor_("can0", 5)
  {
    velocity_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&ControllerTeleop::velocity_callback, this, std::placeholders::_1));

    joystick_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&ControllerTeleop::joy_callback, this, std::placeholders::_1));

    left_actuator_motor_.SetSensorType(SensorType::kEncoder);
    right_actuator_motor_.SetSensorType(SensorType::kEncoder);
    left_actuator_motor_.BurnFlash();
    right_actuator_motor_.BurnFlash();
    RCLCPP_INFO(get_logger(), MAGENTA "MANUAL CONTROL:" RESET " " GREEN "ENABLED" RESET);
    RCLCPP_INFO(get_logger(), "Speed: " YELLOW "SLOW" RESET " (%.1fx)", speed_multiplier_);
  }

private:
  /**
   * @brief Detect rising edge for a button.
   * @param current Current button state.
   * @param previous Previous button state.
   * @return True if button was just pressed.
   */
  bool detect_button_press(bool current, bool& previous)
  {
    bool pressed = current && !previous;
    previous = current;
    return pressed;
  }

  /**
   * @brief Clamp helper for motor duty‑cycle.
   */
  static double clamp(double v)
  {
    return std::clamp(v, -1.0, 1.0);
  }

  /**
   * @brief Drive the robot with the given left and right speeds.
   * @param left_speed Left wheel speed.
   * @param right_speed Right wheel speed.
   */
  void drive(double left_speed, double right_speed)
  {
    left_wheel_motor_.SetDutyCycle(speed_multiplier_ * clamp(left_speed));
    right_wheel_motor_.SetDutyCycle(speed_multiplier_ * clamp(-right_speed));
  }

  /**
   * @brief Process joystick messages in manual mode.
   * @param msg The joystick message.
   */
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Read button and axis values
    bool share_button = msg->buttons[9];  // View button
    bool menu_button = msg->buttons[10];  // Menu button
    bool home_button = msg->buttons[8];   // Xbox button
    bool x_button = msg->buttons[2];      // X button
    bool y_button = msg->buttons[3];      // Y button

    left_joystick_x_ = msg->axes[0];
    left_joystick_y_ = msg->axes[1];
    right_joystick_y_ = -msg->axes[4];

    // Detect button presses (rising edges)
    bool x_pressed = detect_button_press(x_button, prev_x_button_);
    bool y_pressed = detect_button_press(y_button, prev_y_button_);
    bool share_pressed = detect_button_press(share_button, prev_share_button_);
    bool menu_pressed = detect_button_press(menu_button, prev_menu_button_);
    bool home_pressed = detect_button_press(home_button, prev_home_button_);

    // Handle mode switching
    if (share_pressed)
    {
      manual_enabled_ = true;
      RCLCPP_INFO(get_logger(), MAGENTA "MANUAL CONTROL:" RESET " " GREEN "ENABLED" RESET);
    }

    if (menu_pressed)
    {
      manual_enabled_ = false;
      RCLCPP_INFO(get_logger(), YELLOW "AUTONOMOUS CONTROL:" RESET " " GREEN "ENABLED" RESET);
    }

    if (home_pressed)
    {
      robot_disabled_ = !robot_disabled_;
      if (robot_disabled_)
      {
        RCLCPP_ERROR(get_logger(), RED "ROBOT DISABLED" RESET);
      }
      else
      {
        RCLCPP_INFO(get_logger(), GREEN "ROBOT ENABLED" RESET);
      }
    }

    // Toggle vibration motor
    if (x_pressed)
    {
      vibration_enabled_ = !vibration_enabled_;
      RCLCPP_INFO(get_logger(), "Vibration: %s", vibration_enabled_ ? GREEN "ON" RESET : RED "OFF" RESET);
    }

    // Toggle speed multiplier
    if (y_pressed)
    {
      speed_multiplier_ = (speed_multiplier_ < 0.5) ? 0.7 : 0.3;
      const char* speed_label = (speed_multiplier_ >= 0.5) ? "FAST" : "SLOW";
      RCLCPP_INFO(get_logger(), "Speed: " YELLOW "%s" RESET " (%.1fx)", speed_label, speed_multiplier_);
    }

    // Early return if not in manual mode or robot is disabled
    if (!manual_enabled_ || robot_disabled_)
    {
      return;
    }

    // Send heartbeat and control motors
    left_actuator_motor_.Heartbeat();
    right_actuator_motor_.Heartbeat();
    vibration_motor_.Heartbeat();

    // Drive wheels with tank drive
    double left_speed = left_joystick_y_ - left_joystick_x_;
    double right_speed = left_joystick_y_ + left_joystick_x_;
    drive(left_speed, right_speed);

    // Control actuators with right joystick
    double actuator_speed = clamp(right_joystick_y_);
    left_actuator_motor_.SetDutyCycle(actuator_speed);
    right_actuator_motor_.SetDutyCycle(actuator_speed);

    // Control vibration motor
    vibration_motor_.SetDutyCycle(vibration_enabled_ ? 1.0 : 0.0);
  }

  /**
   * @brief Process /cmd_vel in autonomous mode.
   */
  void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (manual_enabled_)
    {
      return;
    }

    // Send heartbeat
    left_actuator_motor_.Heartbeat();

    // Calculate left and right wheel speeds based on linear and angular velocities
    double left_cmd = -0.1 * (msg->linear.x + msg->angular.z * WHEEL_BASE / 2.0) / WHEEL_RADIUS;
    double right_cmd = -0.1 * (msg->linear.x - msg->angular.z * WHEEL_BASE / 2.0) / WHEEL_RADIUS;

    RCLCPP_INFO(get_logger(), "Left: %f, Right: %f", left_cmd, right_cmd);

    // Drive the motors
    drive(left_cmd, right_cmd);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;

  SparkMax left_actuator_motor_, right_actuator_motor_, left_wheel_motor_, right_wheel_motor_, vibration_motor_;

  bool manual_enabled_ = true;
  bool robot_disabled_ = false;
  bool vibration_enabled_ = false;

  // Button states
  bool prev_x_button_ = false;
  bool prev_y_button_ = false;
  bool prev_share_button_ = false;
  bool prev_menu_button_ = false;
  bool prev_home_button_ = false;

  // Speed multiplier
  double speed_multiplier_ = 0.3;

  // Joystick axes
  double left_joystick_x_, left_joystick_y_, right_joystick_y_;
};

/**
 * @brief Main function
 * Initializes and spins the ControllerTeleop node.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
