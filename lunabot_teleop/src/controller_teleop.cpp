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

static constexpr double deposit_ready_pos = 0.6;
static constexpr double excavation_ready_pos = 1.4;

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
    encoder_position_subscriber_ = create_subscription<std_msgs::msg::Float64>(
      "bucket_angle", 10,
      std::bind(&ControllerTeleop::encoder_position_callback, this, std::placeholders::_1));

    manual_mode_publisher_ = create_publisher<std_msgs::msg::Bool>("manual_mode", 10);
    robot_disabled_publisher_ = create_publisher<std_msgs::msg::Bool>("robot_disabled", 10);
    vibration_duty_cycle_publisher_ =
      create_publisher<std_msgs::msg::Float32>("vibration_duty_cycle", 10);
    camera_position_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("/camera_controller/commands", 10);

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
    double drive_sign = camera_inverted_ ? 1.0 : -1.0;
    left_wheel_motor_.SetDutyCycle(
      speed_multiplier_ * clamp(drive_sign * (left_joystick_y_ - left_joystick_x_)));
    right_wheel_motor_.SetDutyCycle(
      speed_multiplier_ * clamp(drive_sign * -(left_joystick_y_ + left_joystick_x_)));

    // Drive actuators: check if moving to target position, otherwise use joystick
    float actuator_cmd = 0.0;
    if (target_position_active_)
    {
      // Use encoder position from absolute encoder
      double error = target_position_ - current_encoder_position_;

      // Check if we've reached the target
      if (std::abs(error) < 0.1)
      {
        target_position_active_ = false;
        actuator_cmd = 0.0;
        LOGGER_SUCCESS(get_logger(), "Reached target bucket position: %.3f rad", target_position_);
      } else
      {
        // Move towards target at max speed
        actuator_cmd = (error > 0) ? 1.0 : -1.0;
      }
    } else
    {
      // Normal joystick control
      actuator_cmd = clamp(right_joystick_y_);
    }

    left_actuator_motor_.SetDutyCycle(actuator_cmd);
    right_actuator_motor_.SetDutyCycle(actuator_cmd);

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

    bool left_paddle_pressed = detect_button_press(msg->buttons[4], prev_left_paddle_);
    bool right_paddle_pressed = detect_button_press(msg->buttons[5], prev_right_paddle_);

    // Plus (+) button enables manual, Minus (-) button enables auto
    bool minus_pressed = detect_button_press(msg->buttons[6], prev_btn_minus_);
    bool plus_pressed = detect_button_press(msg->buttons[7], prev_btn_plus__);

    // D-pad up/down for preset bucket positions
    bool dpad_up_pressed = detect_button_press(msg->axes[7] > 0.5, prev_dpad_up_);
    bool dpad_down_pressed = detect_button_press(msg->axes[7] < -0.5, prev_dpad_down_);

    // D-pad left/right for servo control
    bool dpad_left_pressed = detect_button_press(msg->axes[6] < -0.5, prev_dpad_left_);
    bool dpad_right_pressed = detect_button_press(msg->axes[6] > 0.5, prev_dpad_right_);

    if (share_pressed || plus_pressed)
    {
      manual_enabled_ = true;
      LOGGER_INFO(get_logger(), MAGENTA "Manual control:" RESET " " GREEN "Enabled" RESET);
      publish_state();
    }

    if (menu_pressed || minus_pressed)
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

    if (left_paddle_pressed)
    {
      speed_multiplier_ = (speed_multiplier_ == 0.6f) ? 0.9f : 0.6f;
      LOGGER_INFO(
        get_logger(), "Speed: " YELLOW "%s" RESET " (%.1fx)",
        speed_multiplier_ == 0.9 ? "Turbo" : "Slow", speed_multiplier_);
    }

    if (right_paddle_pressed)
    {
      vibration_enabled_ = !vibration_enabled_;
      LOGGER_INFO(
        get_logger(), "Vibration: %s", vibration_enabled_ ? GREEN "ON" RESET : RED "OFF" RESET);
      publish_state();
    }

    if (dpad_up_pressed)
    {
      target_position_ = deposit_ready_pos;
      target_position_active_ = true;
      LOGGER_INFO(get_logger(), "Moving to deposit bucket position: %.3f rad", deposit_ready_pos);
    }

    if (dpad_down_pressed)
    {
      target_position_ = excavation_ready_pos;
      target_position_active_ = true;
      LOGGER_INFO(
        get_logger(), "Moving to excavation bucket position: %.3f rad", excavation_ready_pos);
    }

    // D-pad left/right to toggle camera servo between 0 and 180 degrees
    if (dpad_left_pressed)
    {
      double target_angle_rad = 0.0;  // 0 degrees
      current_servo_position_ = target_angle_rad;

      auto servo_msg = std_msgs::msg::Float64MultiArray();
      servo_msg.data.push_back(target_angle_rad);
      camera_position_publisher_->publish(servo_msg);

      LOGGER_INFO(get_logger(), "Servo position: 0°");
    }

    if (dpad_right_pressed)
    {
      double target_angle_rad = M_PI;  // 180 degrees
      current_servo_position_ = target_angle_rad;

      auto servo_msg = std_msgs::msg::Float64MultiArray();
      servo_msg.data.push_back(target_angle_rad);
      camera_position_publisher_->publish(servo_msg);

      LOGGER_INFO(get_logger(), "Servo position: 180°");
    }
  }

  /**
   * @brief Inverts drive direction when camera is at 180 degrees.
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
   * @brief Updates current encoder position from encoder_reader.
   * @param msg Float64 message with encoder position in radians.
   */
  void encoder_position_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    current_encoder_position_ = msg->data;
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
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr encoder_position_subscriber_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr manual_mode_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robot_disabled_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr vibration_duty_cycle_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr camera_position_publisher_;

  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  SparkMax left_actuator_motor_, right_actuator_motor_, left_wheel_motor_, right_wheel_motor_,
    vibration_motor_;

  bool manual_enabled_ = true;
  bool robot_disabled_ = false;
  bool vibration_enabled_ = false;
  bool steam_mode_ = false;
  bool camera_inverted_ = false;

  bool prev_left_paddle_ = false;
  bool prev_right_paddle_ = false;
  bool prev_a_button_ = false;
  bool prev_share_button_ = false;
  bool prev_menu_button_ = false;
  bool prev_home_button_ = false;
  bool prev_dpad_up_ = false;
  bool prev_dpad_down_ = false;
  bool prev_dpad_left_ = false;
  bool prev_dpad_right_ = false;
  bool prev_btn_minus_ = false;
  bool prev_btn_plus__ = false;

  float speed_multiplier_ = 0.6f;
  double left_joystick_x_ = 0.0;
  double left_joystick_y_ = 0.0;
  double right_joystick_y_ = 0.0;

  bool target_position_active_ = false;
  double target_position_ = 0.0;
  double current_encoder_position_ = 0.0;
  double current_servo_position_ = 0.0;
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
