/**
 * @file controller_teleop.cpp
 * @author Grayson Arendt
 * @date 4/17/2026
 */

#include "SparkMax.hpp"
#include "lunabot_logger/logger.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>

static constexpr double wheel_radius = 0.095;
static constexpr double wheel_base = 0.53;

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
  : Node("controller_teleop"),
    right_actuator_motor_("can0", 5),
    left_actuator_motor_("can0", 2),
    right_wheel_motor_("can0", 3),
    left_wheel_motor_("can0", 1),
    vibration_motor_("can0", 4)
  {
    declare_parameter("steam_mode", false);
    get_parameter("steam_mode", steam_mode_);

    LOGGER_INFO(get_logger(), "Controller mode: %s", steam_mode_ ? "Steam Deck" : "Xbox");

    velocity_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&ControllerTeleop::velocity_callback, this, std::placeholders::_1));
    joystick_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&ControllerTeleop::joy_callback, this, std::placeholders::_1));
    emergency_stop_subscriber_ = create_subscription<std_msgs::msg::Bool>(
      "emergency_stop", 10,
      std::bind(&ControllerTeleop::emergency_stop_callback, this, std::placeholders::_1));
    mode_switch_subscriber_ = create_subscription<std_msgs::msg::Bool>(
      "mode_switch", 10,
      std::bind(&ControllerTeleop::mode_switch_callback, this, std::placeholders::_1));
    camera_position_subscriber_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/camera_controller/commands", 10,
      std::bind(&ControllerTeleop::camera_position_callback, this, std::placeholders::_1));
    home_offset_subscriber_ = create_subscription<std_msgs::msg::Float64>(
      "actuator_home_offset", 10,
      std::bind(&ControllerTeleop::home_offset_callback, this, std::placeholders::_1));

    manual_mode_publisher_ = create_publisher<std_msgs::msg::Bool>("manual_mode", 10);
    robot_disabled_publisher_ = create_publisher<std_msgs::msg::Bool>("robot_disabled", 10);
    vibration_duty_cycle_publisher_ =
      create_publisher<std_msgs::msg::Float32>("vibration_duty_cycle", 10);
    home_offset_publisher_ = create_publisher<std_msgs::msg::Float64>("actuator_home_offset", 10);
    actuator_position_publisher_ =
      create_publisher<std_msgs::msg::Float64>("actuator_position", 10);

    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&ControllerTeleop::publish_state, this));
    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&ControllerTeleop::control_loop, this));

    left_actuator_motor_.SetSensorType(SensorType::kEncoder);
    right_actuator_motor_.SetSensorType(SensorType::kEncoder);
    left_actuator_motor_.SetIdleMode(IdleMode::kBrake);
    right_actuator_motor_.SetIdleMode(IdleMode::kBrake);
    left_actuator_motor_.BurnFlash();
    right_actuator_motor_.BurnFlash();

    LOGGER_INFO(get_logger(), MAGENTA "Manual control:" RESET " " GREEN "Enabled" RESET);
    LOGGER_INFO(get_logger(), "Speed: " YELLOW "Slow" RESET " (%.1fx)", speed_multiplier_);
  }

private:
  /**
   * @brief Detects a rising edge.
   * @param current Current button state.
   * @param previous Previous button state.
   * @return True if button was just pressed.
   */
  static bool detect_button_press(bool current, bool & previous)
  {
    bool pressed = current && !previous;
    previous = current;
    return pressed;
  }

  /**
   * @brief Clamps a value to [-1, 1] for motor duty cycle.
   */
  static float clamp(double v) { return static_cast<float>(std::clamp(v, -1.0, 1.0)); }

  /**
   * @brief Returns the correct button index for the active controller.
   * @param steam_idx Button index for Steam Deck.
   * @param xbox_idx Button index for Xbox.
   */
  int get_button_index(int steam_idx, int xbox_idx) const
  {
    return steam_mode_ ? steam_idx : xbox_idx;
  }

  /**
   * @brief Motor control loop running at 20 Hz.
   */
  void control_loop()
  {
    left_actuator_motor_.Heartbeat();

    // Always publish actuator position so the GUI can read bucket state
    auto pos_msg = std_msgs::msg::Float64();
    pos_msg.data = left_actuator_motor_.GetPosition() - actuator_zero_offset_;
    actuator_position_publisher_->publish(pos_msg);

    if (robot_disabled_)
    {
      left_wheel_motor_.SetDutyCycle(0.0);
      right_wheel_motor_.SetDutyCycle(0.0);
      left_actuator_motor_.SetDutyCycle(0.0);
      right_actuator_motor_.SetDutyCycle(0.0);
      vibration_motor_.SetDutyCycle(0.0);
      return;
    }

    if (!manual_enabled_) return;

    // Drive wheels with left joystick (invert direction when fisheye camera is at 180 degrees)
    double drive_sign = camera_inverted_ ? -1.0 : 1.0;
    left_wheel_motor_.SetDutyCycle(
      speed_multiplier_ * clamp(drive_sign * (left_joystick_y_ - left_joystick_x_)));
    right_wheel_motor_.SetDutyCycle(
      speed_multiplier_ * clamp(drive_sign * -(left_joystick_y_ + left_joystick_x_)));

    // Drive actuators with right joystick
    float actuator_cmd = clamp(right_joystick_y_);
    left_actuator_motor_.SetDutyCycle(actuator_cmd);
    right_actuator_motor_.SetDutyCycle(actuator_cmd);

    LOGGER_INFO(
      get_logger(), "Actuator positions - Left: %.6f, Right: %.6f",
      left_actuator_motor_.GetPosition() - actuator_zero_offset_,
      right_actuator_motor_.GetPosition() - actuator_zero_offset_);

    vibration_motor_.SetDutyCycle(vibration_enabled_ ? 0.5 : 0.0);
  }

  /**
   * @brief Processes joystick input in manual mode.
   * @param msg The joystick message.
   */
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // Detect button presses (rising edges)
    bool share_pressed =
      detect_button_press(msg->buttons[get_button_index(2, 9)], prev_share_button_);
    bool menu_pressed =
      detect_button_press(msg->buttons[get_button_index(14, 10)], prev_menu_button_);
    bool home_pressed =
      detect_button_press(msg->buttons[get_button_index(11, 8)], prev_home_button_);
    bool x_pressed = detect_button_press(msg->buttons[get_button_index(5, 2)], prev_x_button_);
    bool y_pressed = detect_button_press(msg->buttons[get_button_index(6, 3)], prev_y_button_);
    bool a_pressed = detect_button_press(msg->buttons[0], prev_a_button_);

    if (share_pressed)
    {
      manual_enabled_ = true;
      LOGGER_INFO(get_logger(), MAGENTA "Manual control:" RESET " " GREEN "Enabled" RESET);
      publish_state();
    }

    if (menu_pressed)
    {
      manual_enabled_ = false;
      vibration_enabled_ = false;
      left_joystick_x_ = left_joystick_y_ = right_joystick_y_ = 0.0;
      LOGGER_INFO(get_logger(), YELLOW "Autonomous control:" RESET " " GREEN "Enabled" RESET);
      publish_state();
    }

    if (home_pressed)
    {
      robot_disabled_ = !robot_disabled_;
      if (robot_disabled_)
      {
        LOGGER_FAILURE(get_logger(), "Robot disabled");
      } else
      {
        LOGGER_SUCCESS(get_logger(), "Robot enabled");
      }
      publish_state();
    }

    if (!manual_enabled_) return;

    // Read joystick axes
    left_joystick_x_ = msg->axes[0];
    left_joystick_y_ = msg->axes[1];
    right_joystick_y_ = -(steam_mode_ ? msg->axes[3] : msg->axes[4]);

    if (x_pressed)
    {
      vibration_enabled_ = !vibration_enabled_;
      LOGGER_INFO(
        get_logger(), "Vibration: %s", vibration_enabled_ ? GREEN "ON" RESET : RED "OFF" RESET);
      publish_state();
    }

    if (y_pressed)
    {
      speed_multiplier_ = (speed_multiplier_ == 0.6f) ? 0.9f : 0.6f;
      LOGGER_INFO(
        get_logger(), "Speed: " YELLOW "%s" RESET " (%.1fx)",
        speed_multiplier_ == 0.9 ? "Turbo" : "Slow", speed_multiplier_);
    }

    if (a_pressed)
    {
      actuator_zero_offset_ = left_actuator_motor_.GetPosition();
      auto offset_msg = std_msgs::msg::Float64();
      offset_msg.data = actuator_zero_offset_;
      home_offset_publisher_->publish(offset_msg);
      LOGGER_SUCCESS(get_logger(), "Actuator zeroed at position: %.6f", actuator_zero_offset_);
    }
  }

  /**
   * @brief Inverts drive direction when camera is at 180 degrees (pi radians).
   * @param msg Float64MultiArray with camera angle in radians.
   */
  void camera_position_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.empty()) return;

    camera_inverted_ = (std::abs(msg->data[0] - M_PI) < 0.1);
    LOGGER_INFO(
      get_logger(), "Camera position: %.1f°: driving %s", std::abs(msg->data[0]) * 180.0 / M_PI,
      camera_inverted_ ? "INVERTED" : "NORMAL");
  }

  /**
   * @brief Processes cmd_vel in autonomous mode.
   * @param msg Twist message with linear and angular velocity.
   */
  void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (manual_enabled_) return;

    double left_cmd = -0.1 * (msg->linear.x + msg->angular.z * wheel_base / 2.0) / wheel_radius;
    double right_cmd = -0.1 * (msg->linear.x - msg->angular.z * wheel_base / 2.0) / wheel_radius;

    left_wheel_motor_.SetDutyCycle(clamp(left_cmd));
    right_wheel_motor_.SetDutyCycle(clamp(-right_cmd));
  }

  /**
   * @brief Immediately stops all motors on emergency stop.
   * @param msg Bool message; true triggers the stop.
   */
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) return;

    robot_disabled_ = true;
    LOGGER_FAILURE(get_logger(), "Emergency stop activated!");
    left_wheel_motor_.SetDutyCycle(0.0);
    right_wheel_motor_.SetDutyCycle(0.0);
    left_actuator_motor_.SetDutyCycle(0.0);
    right_actuator_motor_.SetDutyCycle(0.0);
    vibration_motor_.SetDutyCycle(0.0);
    publish_state();
  }

  /**
   * @brief Switches between manual and autonomous mode from the GUI.
   * @param msg Bool message; true = manual, false = autonomous.
   */
  void mode_switch_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    manual_enabled_ = msg->data;
    if (manual_enabled_)
    {
      LOGGER_INFO(get_logger(), MAGENTA "Manual control:" RESET " " GREEN "Enabled" RESET);
    } else
    {
      LOGGER_INFO(get_logger(), YELLOW "Autonomous control:" RESET " " GREEN "Enabled" RESET);
    }
    publish_state();
  }

  /**
   * @brief Updates the actuator zero offset.
   * @param msg Float64 message with the new offset value.
   */
  void home_offset_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    actuator_zero_offset_ = msg->data;
    LOGGER_SUCCESS(get_logger(), "Actuator offset updated: %.6f", actuator_zero_offset_);
  }

  /**
   * @brief Publishes manual mode, disabled state, and vibration duty cycle.
   */
  void publish_state()
  {
    auto manual_msg = std_msgs::msg::Bool();
    manual_msg.data = manual_enabled_;
    manual_mode_publisher_->publish(manual_msg);

    auto disabled_msg = std_msgs::msg::Bool();
    disabled_msg.data = robot_disabled_;
    robot_disabled_publisher_->publish(disabled_msg);

    auto duty_msg = std_msgs::msg::Float32();
    duty_msg.data = vibration_enabled_ ? 1.0f : 0.0f;
    vibration_duty_cycle_publisher_->publish(duty_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mode_switch_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr camera_position_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr home_offset_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_mode_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robot_disabled_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vibration_duty_cycle_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr home_offset_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr actuator_position_publisher_;

  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  SparkMax left_actuator_motor_, right_actuator_motor_, left_wheel_motor_, right_wheel_motor_,
    vibration_motor_;

  bool manual_enabled_ = true;
  bool robot_disabled_ = false;
  bool vibration_enabled_ = false;
  bool steam_mode_ = false;
  bool camera_inverted_ = false;

  bool prev_x_button_ = false;
  bool prev_y_button_ = false;
  bool prev_a_button_ = false;
  bool prev_share_button_ = false;
  bool prev_menu_button_ = false;
  bool prev_home_button_ = false;

  float speed_multiplier_ = 0.6f;
  double actuator_zero_offset_ = 0.0;
  double left_joystick_x_ = 0.0;
  double left_joystick_y_ = 0.0;
  double right_joystick_y_ = 0.0;
};

/**
 * @brief Initializes and spins the ControllerTeleop node.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerTeleop>());
  rclcpp::shutdown();
  return 0;
}
