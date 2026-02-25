/**
 * @file controller_teleop.cpp
 * @author Grayson Arendt
 * @date 4/17/2025
 */

#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

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

    emergency_stop_subscriber_ = create_subscription<std_msgs::msg::Bool>(
        "emergency_stop", 10, std::bind(&ControllerTeleop::emergency_stop_callback, this, std::placeholders::_1));

    // Subscribe to mode switch commands from GUI
    mode_switch_subscriber_ = create_subscription<std_msgs::msg::Bool>(
        "mode_switch", 10, std::bind(&ControllerTeleop::mode_switch_callback, this, std::placeholders::_1));

    // Publishers for robot state
    manual_mode_publisher_ = create_publisher<std_msgs::msg::Bool>("manual_mode", 10);
    robot_disabled_publisher_ = create_publisher<std_msgs::msg::Bool>("robot_disabled", 10);
    vibration_duty_cycle_publisher_ = create_publisher<std_msgs::msg::Float32>("vibration_duty_cycle", 10);

    // Timer to publish state periodically
    state_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ControllerTeleop::publish_state, this));

    // Timer for motor control at fixed rate 
    control_timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&ControllerTeleop::control_loop, this));

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
   * @brief Control loop at fixed rate
   */
  void control_loop()
  {
    // Send heartbeats
    left_actuator_motor_.Heartbeat();
    right_actuator_motor_.Heartbeat();
    vibration_motor_.Heartbeat();
    left_wheel_motor_.Heartbeat();
    right_wheel_motor_.Heartbeat();

    if (robot_disabled_)
    {
      // Stop all motors if robot is disabled
      left_wheel_motor_.SetDutyCycle(0.0);
      right_wheel_motor_.SetDutyCycle(0.0);
      left_actuator_motor_.SetDutyCycle(0.0);
      right_actuator_motor_.SetDutyCycle(0.0);
      vibration_motor_.SetDutyCycle(0.0);
    }
    else if (manual_enabled_)
    {
      // Control wheel motors with left joystick
      double left_speed = left_joystick_y_ - left_joystick_x_;
      double right_speed = left_joystick_y_ + left_joystick_x_;
      left_wheel_motor_.SetDutyCycle(speed_multiplier_ * clamp(left_speed));
      right_wheel_motor_.SetDutyCycle(speed_multiplier_ * clamp(-right_speed));

      // Control actuators with right joystick
      double actuator_speed = clamp(right_joystick_y_);
      left_actuator_motor_.SetDutyCycle(actuator_speed);
      right_actuator_motor_.SetDutyCycle(actuator_speed);

      // Control vibration motor
      vibration_motor_.SetDutyCycle(vibration_enabled_ ? 1.0 : 0.0);
    }
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
      publish_state();
    }

    if (menu_pressed)
    {
      manual_enabled_ = false;
      // Clear vibration state when switching to auto mode (GUI update only)
      vibration_enabled_ = false;
      RCLCPP_INFO(get_logger(), YELLOW "AUTONOMOUS CONTROL:" RESET " " GREEN "ENABLED" RESET);
      publish_state();
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
      publish_state();
    }

    // Toggle vibration motor and read back duty cycle (manual mode only)
    if (x_pressed && manual_enabled_)
    {
      vibration_enabled_ = !vibration_enabled_;
      RCLCPP_INFO(get_logger(), "Vibration: %s", vibration_enabled_ ? GREEN "ON" RESET : RED "OFF" RESET);
      publish_state();  // Publish updated duty cycle immediately
    }

    // Toggle speed multiplier
    if (y_pressed)
    {
      speed_multiplier_ = (speed_multiplier_ < 0.5) ? 0.7 : 0.3;
      const char* speed_label = (speed_multiplier_ >= 0.5) ? "FAST" : "SLOW";
      RCLCPP_INFO(get_logger(), "Speed: " YELLOW "%s" RESET " (%.1fx)", speed_label, speed_multiplier_);
    }
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

    // Calculate left and right wheel speeds based on linear and angular velocities
    double left_cmd = -0.1 * (msg->linear.x + msg->angular.z * WHEEL_BASE / 2.0) / WHEEL_RADIUS;
    double right_cmd = -0.1 * (msg->linear.x - msg->angular.z * WHEEL_BASE / 2.0) / WHEEL_RADIUS;

    RCLCPP_INFO(get_logger(), "Left: %f, Right: %f", left_cmd, right_cmd);

    // Send motor commands directly in autonomous mode
    left_wheel_motor_.SetDutyCycle(clamp(left_cmd));
    right_wheel_motor_.SetDutyCycle(clamp(-right_cmd));
  }

  /**
   * @brief Process emergency stop command.
   */
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      robot_disabled_ = true;
      RCLCPP_ERROR(get_logger(), RED "EMERGENCY STOP ACTIVATED!" RESET);
      // Stop all motors immediately
      left_wheel_motor_.SetDutyCycle(0.0);
      right_wheel_motor_.SetDutyCycle(0.0);
      left_actuator_motor_.SetDutyCycle(0.0);
      right_actuator_motor_.SetDutyCycle(0.0);
      vibration_motor_.SetDutyCycle(0.0);
      publish_state();
    }
  }

  /**
   * @brief Process mode switch command from GUI.
   */
  void mode_switch_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    manual_enabled_ = msg->data;
    if (manual_enabled_)
    {
      RCLCPP_INFO(get_logger(), MAGENTA "MANUAL CONTROL:" RESET " " GREEN "ENABLED" RESET);
    }
    else
    {
      RCLCPP_INFO(get_logger(), YELLOW "AUTONOMOUS CONTROL:" RESET " " GREEN "ENABLED" RESET);
      // Clear vibration state when switching to auto mode
      vibration_enabled_ = false;
    }
    publish_state();
  }

  /**
   * @brief Publish robot state (manual mode and disabled status).
   */
  void publish_state()
  {
    auto manual_msg = std_msgs::msg::Bool();
    manual_msg.data = manual_enabled_;
    manual_mode_publisher_->publish(manual_msg);

    auto disabled_msg = std_msgs::msg::Bool();
    disabled_msg.data = robot_disabled_;
    robot_disabled_publisher_->publish(disabled_msg);

    // Publish vibration duty cycle (0.0 if off, 1.0 if on)
    auto duty_msg = std_msgs::msg::Float32();
    duty_msg.data = vibration_enabled_ ? 1.0 : 0.0;
    vibration_duty_cycle_publisher_->publish(duty_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_switch_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_mode_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robot_disabled_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vibration_duty_cycle_publisher_;

  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;

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
  double left_joystick_x_ = 0.0;
  double left_joystick_y_ = 0.0;
  double right_joystick_y_ = 0.0;
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
