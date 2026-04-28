/**
 * @file controller_teleop.cpp
 * @author Grayson Arendt
 * @date 4/17/2026
 */

#include "SparkMax.hpp"
#include "lunabot_logger/logger.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <lunabot_msgs/action/depositing.hpp>
#include <lunabot_msgs/action/excavation.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <memory>

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
  ControllerTeleop() : Node("controller_teleop")
  {
    declare_parameter("steam_mode", false);
    get_parameter("steam_mode", steam_mode_);

    LOGGER_INFO(get_logger(), "Controller mode: %s", steam_mode_ ? "Steam Deck" : "Xbox");

    try
    {
      right_actuator_motor_ = std::make_unique<SparkMax>("can0", 5);
      left_actuator_motor_ = std::make_unique<SparkMax>("can0", 2);
      right_wheel_motor_ = std::make_unique<SparkMax>("can0", 3);
      left_wheel_motor_ = std::make_unique<SparkMax>("can0", 1);
      vibration_motor_ = std::make_unique<SparkMax>("can0", 4);

      left_actuator_motor_->SetSensorType(SensorType::kEncoder);
      right_actuator_motor_->SetSensorType(SensorType::kEncoder);
      left_actuator_motor_->SetIdleMode(IdleMode::kBrake);
      right_actuator_motor_->SetIdleMode(IdleMode::kBrake);
      left_actuator_motor_->BurnFlash();
      right_actuator_motor_->BurnFlash();

      motors_available_ = true;
      LOGGER_SUCCESS(get_logger(), "Motor controllers initialized successfully");
    } catch (const std::exception & e)
    {
      motors_available_ = false;
      LOGGER_WARN(
        get_logger(),
        YELLOW "Motor controllers not available: %s" RESET "\n" CYAN
               "Running in simulation mode." RESET,
        e.what());
    }

    velocity_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&ControllerTeleop::velocity_callback, this, std::placeholders::_1));
    joystick_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&ControllerTeleop::joy_callback, this, std::placeholders::_1));
    emergency_stop_subscriber_ = create_subscription<std_msgs::msg::Bool>(
      "/emergency_stop", 10,
      std::bind(&ControllerTeleop::emergency_stop_callback, this, std::placeholders::_1));
    mode_switch_subscriber_ = create_subscription<std_msgs::msg::Bool>(
      "/mode_switch", 10,
      std::bind(&ControllerTeleop::mode_switch_callback, this, std::placeholders::_1));
    camera_position_subscriber_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/camera_controller/commands", 10,
      std::bind(&ControllerTeleop::camera_position_callback, this, std::placeholders::_1));
    encoder_position_subscriber_ = create_subscription<std_msgs::msg::Float64>(
      "/bucket_angle", 10,
      std::bind(&ControllerTeleop::encoder_position_callback, this, std::placeholders::_1));

    manual_mode_publisher_ = create_publisher<std_msgs::msg::Bool>("/manual_mode", 10);
    robot_disabled_publisher_ = create_publisher<std_msgs::msg::Bool>("/robot_disabled", 10);
    vibration_duty_cycle_publisher_ =
      create_publisher<std_msgs::msg::Float32>("/vibration_duty_cycle", 10);
    camera_position_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("/camera_controller/commands", 10);

    excavation_client_ =
      rclcpp_action::create_client<lunabot_msgs::action::Excavation>(this, "excavation_action");
    depositing_client_ =
      rclcpp_action::create_client<lunabot_msgs::action::Depositing>(this, "depositing_action");

    state_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&ControllerTeleop::publish_state, this));
    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&ControllerTeleop::control_loop, this));

    LOGGER_INFO(get_logger(), MAGENTA "Manual control:" RESET " " GREEN "Enabled" RESET);
    LOGGER_INFO(
      get_logger(), "Speed: %s (%.1fx)",
      speed_multiplier_ == 0.9f ? RED "Turbo" RESET : YELLOW "Slow" RESET, speed_multiplier_);
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
    if (!motors_available_) return;

    left_actuator_motor_->Heartbeat();

    if (robot_disabled_)
    {
      left_wheel_motor_->SetDutyCycle(0.0);
      right_wheel_motor_->SetDutyCycle(0.0);
      left_actuator_motor_->SetDutyCycle(0.0);
      right_actuator_motor_->SetDutyCycle(0.0);
      vibration_motor_->SetDutyCycle(0.0);
      return;
    }

    if (!manual_enabled_) return;

    // Drive wheels with left joystick
    double linear_sign = camera_inverted_ ? 1.0 : -1.0;
    double linear = linear_sign * left_joystick_y_;
    double rotation = left_joystick_x_;

    left_wheel_motor_->SetDutyCycle(speed_multiplier_ * clamp(linear - rotation));
    right_wheel_motor_->SetDutyCycle(speed_multiplier_ * clamp(-(linear + rotation)));

    // Check if moving to target position, otherwise use joystick
    float actuator_cmd = 0.0;
    if (target_position_active_)
    {
      // Use encoder position
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

    left_actuator_motor_->SetDutyCycle(actuator_cmd);
    right_actuator_motor_->SetDutyCycle(actuator_cmd);

    vibration_motor_->SetDutyCycle(vibration_enabled_ ? 1.0 : 0.0);
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
      detect_button_press(msg->buttons[get_button_index(11, 12)], prev_home_button_);

    bool left_paddle_pressed = detect_button_press(msg->buttons[5], prev_left_paddle_);
    bool right_paddle_pressed = detect_button_press(msg->buttons[2], prev_right_paddle_);

    // Plus (+) button enables manual, Minus (-) button enables auto
    bool minus_pressed = detect_button_press(msg->buttons[10], prev_btn_minus_);
    bool plus_pressed = detect_button_press(msg->buttons[11], prev_btn_plus__);

    // D-pad up/down for preset bucket positions
    bool dpad_up_pressed = detect_button_press(msg->axes[7] > 0.5, prev_dpad_up_);
    bool dpad_down_pressed = detect_button_press(msg->axes[7] < -0.5, prev_dpad_down_);

    // L4/R4 buttons for camera servo control
    bool l4_pressed = detect_button_press(msg->buttons[16], prev_l4_);
    bool r4_pressed = detect_button_press(msg->buttons[17], prev_r4_);

    // X/Y buttons for excavate/deposit actions
    bool x_pressed = detect_button_press(msg->buttons[3], prev_x_button_);
    bool y_pressed = detect_button_press(msg->buttons[4], prev_y_button_);

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
        get_logger(), "Speed: %s (%.1fx)",
        speed_multiplier_ == 0.9f ? RED "Turbo" RESET : YELLOW "Slow" RESET, speed_multiplier_);
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

    if (l4_pressed)
    {
      double target_angle_rad = 0.0;
      current_servo_position_ = target_angle_rad;

      auto servo_msg = std_msgs::msg::Float64MultiArray();
      servo_msg.data.push_back(target_angle_rad);
      camera_position_publisher_->publish(servo_msg);

      LOGGER_INFO(get_logger(), "Servo position: 0°");
    }

    if (r4_pressed)
    {
      double target_angle_rad = M_PI;
      current_servo_position_ = target_angle_rad;

      auto servo_msg = std_msgs::msg::Float64MultiArray();
      servo_msg.data.push_back(target_angle_rad);
      camera_position_publisher_->publish(servo_msg);

      LOGGER_INFO(get_logger(), "Servo position: 180°");
    }

    if (x_pressed)
    {
      send_excavate_goal();
    }

    if (y_pressed)
    {
      send_deposit_goal();
    }
  }

  void send_excavate_goal()
  {
    if (!excavation_client_->wait_for_action_server(std::chrono::seconds(0)))
    {
      LOGGER_WARN(get_logger(), "Excavation action server not available");
      return;
    }

    auto goal_msg = lunabot_msgs::action::Excavation::Goal();
    LOGGER_INFO(get_logger(), CYAN "Sending excavation goal" RESET);

    auto send_goal_options =
      rclcpp_action::Client<lunabot_msgs::action::Excavation>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](auto goal_handle) {
      if (!goal_handle)
      {
        LOGGER_FAILURE(get_logger(), "Excavation goal rejected");
      } else
      {
        LOGGER_SUCCESS(get_logger(), "Excavation goal accepted");
      }
    };
    send_goal_options.result_callback = [this](const auto & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
      {
        LOGGER_SUCCESS(get_logger(), "Excavation completed");
      } else
      {
        LOGGER_FAILURE(get_logger(), "Excavation failed");
      }
    };

    excavation_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void send_deposit_goal()
  {
    if (!depositing_client_->wait_for_action_server(std::chrono::seconds(0)))
    {
      LOGGER_WARN(get_logger(), "Depositing action server not available");
      return;
    }

    auto goal_msg = lunabot_msgs::action::Depositing::Goal();
    LOGGER_INFO(get_logger(), CYAN "Sending deposit goal" RESET);

    auto send_goal_options =
      rclcpp_action::Client<lunabot_msgs::action::Depositing>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](auto goal_handle) {
      if (!goal_handle)
      {
        LOGGER_FAILURE(get_logger(), "Deposit goal rejected");
      } else
      {
        LOGGER_SUCCESS(get_logger(), "Deposit goal accepted");
      }
    };
    send_goal_options.result_callback = [this](const auto & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
      {
        LOGGER_SUCCESS(get_logger(), "Deposit completed");
      } else
      {
        LOGGER_FAILURE(get_logger(), "Deposit failed");
      }
    };

    depositing_client_->async_send_goal(goal_msg, send_goal_options);
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
    if (!motors_available_ || manual_enabled_) return;

    double left_cmd = -0.1 * (msg->linear.x + msg->angular.z * wheel_base / 2.0) / wheel_radius;
    double right_cmd = -0.1 * (msg->linear.x - msg->angular.z * wheel_base / 2.0) / wheel_radius;

    left_wheel_motor_->SetDutyCycle(clamp(left_cmd));
    right_wheel_motor_->SetDutyCycle(clamp(-right_cmd));
  }

  /**
   * @brief Immediately stops all motors on emergency stop.
   * @param msg Bool message; true triggers the stop, false re-enables.
   */
  void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      robot_disabled_ = true;
      LOGGER_FAILURE(get_logger(), "Emergency stop activated!");

      if (motors_available_)
      {
        left_wheel_motor_->SetDutyCycle(0.0);
        right_wheel_motor_->SetDutyCycle(0.0);
        left_actuator_motor_->SetDutyCycle(0.0);
        right_actuator_motor_->SetDutyCycle(0.0);
        vibration_motor_->SetDutyCycle(0.0);
      }
    } else
    {
      robot_disabled_ = false;
      LOGGER_SUCCESS(get_logger(), "Robot re-enabled from emergency stop");
    }

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

  rclcpp_action::Client<lunabot_msgs::action::Excavation>::SharedPtr excavation_client_;
  rclcpp_action::Client<lunabot_msgs::action::Depositing>::SharedPtr depositing_client_;

  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::unique_ptr<SparkMax> left_actuator_motor_;
  std::unique_ptr<SparkMax> right_actuator_motor_;
  std::unique_ptr<SparkMax> left_wheel_motor_;
  std::unique_ptr<SparkMax> right_wheel_motor_;
  std::unique_ptr<SparkMax> vibration_motor_;

  bool motors_available_ = false;

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
  bool prev_l4_ = false;
  bool prev_r4_ = false;
  bool prev_x_button_ = false;
  bool prev_y_button_ = false;
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
