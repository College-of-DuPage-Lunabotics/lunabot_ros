/**
 * @file controller_teleop.cpp
 * @author Grayson Arendt
 * @date 4/17/2025
 */

#include <algorithm>
#include <array>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "SparkMax.hpp"

/**
 * @class ControllerTeleop
 * @brief Handles joystick/navigation commands, applies smoothing, and drives motors.
 */
class ControllerTeleop : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for ControllerTeleop.
   */
  ControllerTeleop()
  : Node("controller_teleop"),
    left_wheel_motor_("can0", 1),
    right_wheel_motor_("can0", 2),
    lift_actuator_motor_("can0", 3)
  {
    declare_and_get_parameters();

    velocity_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&ControllerTeleop::velocity_callback, this, std::placeholders::_1));

    joystick_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&ControllerTeleop::joy_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "\033[1;35mMANUAL CONTROL:\033[0m \033[1;32mENABLED\033[0m");
  }

private:
  /**
   * @brief Declare parameters and store them in variables.
   */
  void declare_and_get_parameters()
  {
    declare_parameter("smoothing_alpha", 0.2);
    declare_parameter("steam_mode", true);
    declare_parameter("xbox_mode", false);
    declare_parameter("ps4_mode", false);
    declare_parameter("switch_mode", false);

    get_parameter("smoothing_alpha", smoothing_alpha_);
    get_parameter("steam_mode", steam_mode_);
    get_parameter("xbox_mode", xbox_mode_);
    get_parameter("ps4_mode", ps4_mode_);
    get_parameter("switch_mode", switch_mode_);
  }

  /**
   * @brief Clamp helper for motor duty‑cycle.
   */
  static double clamp(double v) {return std::clamp(v, -1.0, 1.0);}

  /**
   * @brief Low‑pass filter for wheel speed commands.
   * @param curr Current (unsmoothed) command.
   * @param prev Previous filtered value (updated).
   * @return Smoothed and clamped command.
   */
  double smooth_speed(double curr, double & prev)
  {
    curr = smoothing_alpha_ * curr + (1.0 - smoothing_alpha_) * prev;
    prev = curr;
    return clamp(curr);
  }

  /**
   * @brief Retrieve a button index based on controller type.

   * @param msg The joystick message.
   * @param map The mapping of buttons for different controllers.
   * @return The button state.
   */
  int get_button(
    const sensor_msgs::msg::Joy::SharedPtr & msg,
    const std::array<int, 4> & map) const
  {
    size_t idx = steam_mode_ ? 3 : xbox_mode_ ? 2 : ps4_mode_ ? 1 : 0;
    return msg->buttons[map[idx]];
  }

  /**
   * @brief Retrieve an axis index based on controller type.
   * @param msg The joystick message.
   * @param map The mapping of axes for different controllers.
   * @return The axis value.
   */
  double get_axis(
    const sensor_msgs::msg::Joy::SharedPtr & msg,
    const std::array<int, 3> & map) const
  {
    size_t idx = xbox_mode_ ? 2 : ps4_mode_ ? 1 : 0;
    return msg->axes[map[idx]];
  }

  /**
   * @brief Drive the robot with the given left and right speeds.
   * @param left_speed Left wheel speed.
   * @param right_speed Right wheel speed.
   */
  void drive(double left_speed, double right_speed)
  {
    if (abs(left_speed) <= 0.1 && abs(right_speed) <= 0.1) {
      left_wheel_motor_.SetDutyCycle(0.0);
      right_wheel_motor_.SetDutyCycle(0.0);
      return;
    }

    left_wheel_motor_.SetDutyCycle(
      smooth_speed(left_speed, prev_left_speed_));

    right_wheel_motor_.SetDutyCycle(
      smooth_speed(right_speed, prev_right_speed_));
  }

  /**
   * @brief Process joystick messages in manual mode.
   * @param msg The joystick message.
   */
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    auto clock = rclcpp::Clock();

    share_button_ = get_button(msg, {9, 8, 9, 2});
    menu_button_ = get_button(msg, {10, 9, 10, 13});
    home_button_ = get_button(msg, {11, 10, 8, 11});
    x_button_ = get_button(msg, {2, 3, 2, 5});
    y_button_ = get_button(msg, {3, 2, 3, 6});

    left_joystick_x_ = msg->axes[0];
    left_joystick_y_ = msg->axes[1];
    right_joystick_y_ = -msg->axes[3];

    d_pad_vertical_ = get_axis(msg, {5, 7, 7});
  
    if (share_button_) {
      manual_enabled_ = true;
      RCLCPP_INFO_THROTTLE(
        get_logger(), clock, 1000, "\033[1;35mMANUAL CONTROL:\033[0m \033[1;32mENABLED\033[0m");
    }

    if (menu_button_) {
      manual_enabled_ = false;
      RCLCPP_INFO_THROTTLE(
        get_logger(), clock, 1000, "\033[1;33mAUTONOMOUS CONTROL:\033[0m \033[1;32mENABLED\033[0m");
    }

    if (home_button_) {
      robot_disabled_ = true;
      RCLCPP_ERROR(get_logger(), "\033[1;31mROBOT DISABLED\033[0m");
    }

    if (!manual_enabled_ || robot_disabled_) {return;}

    lift_actuator_motor_.Heartbeat();

    double drive_speed_multiplier = (x_button_) ? 1.0 : 0.7;

    double left_speed = left_joystick_y_ - left_joystick_x_;
    double right_speed = left_joystick_y_ + left_joystick_x_;

    // Apply smoothing to the commands and drive the motors
    drive(left_speed * drive_speed_multiplier, right_speed * drive_speed_multiplier);

    // Lift/lower blade usnig right joystick
    double lift_speed = right_joystick_y_;
    lift_speed = clamp(lift_speed);

    if (abs(lift_speed) > 0.4) {
      lift_actuator_motor_.SetDutyCycle(lift_speed);
      }
    else {
      lift_actuator_motor_.SetDutyCycle(0.0);
    }
  }

  /**
   * @brief Process /cmd_vel in autonomous mode.
   */
  void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (manual_enabled_) {return;}
    lift_actuator_motor_.Heartbeat();

    // Calculate left and right wheel speeds based on linear and angular velocities
    double wheel_radius = 0.095;
    double wheel_base = 0.53;

    double left_cmd = -0.1 * (msg->linear.x + msg->angular.z * wheel_base / 2.0) / wheel_radius;
    double right_cmd = -0.1 * (msg->linear.x - msg->angular.z * wheel_base / 2.0) / wheel_radius;

    RCLCPP_INFO(
      get_logger(), "Left: %f, Right: %f", left_cmd, right_cmd);

    // Apply smoothing to the commands and drive the motors
    drive(left_cmd, right_cmd);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;

  SparkMax left_wheel_motor_, right_wheel_motor_, lift_actuator_motor_;

  bool manual_enabled_ = false, robot_disabled_ = false;
  bool steam_mode_, xbox_mode_, ps4_mode_, switch_mode_;
  bool share_button_, menu_button_, home_button_, x_button_, y_button_;

  double d_pad_vertical_;
  double left_joystick_x_, left_joystick_y_, right_joystick_y_;
  double prev_left_speed_ = 0.0, prev_right_speed_ = 0.0;
  double smoothing_alpha_;
};

/**
 * @brief Main function
 * Initializes and spins the ControllerTeleop node.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControllerTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
