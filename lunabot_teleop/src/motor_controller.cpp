/**
 * @file motor_controller.cpp
 * @author Grayson Arendt
 * @date 4/28/2026
 */

#include "SparkMax.hpp"
#include "lunabot_logger/logger.hpp"

#include <rclcpp/rclcpp.hpp>

#include <lunabot_msgs/msg/motor_commands.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <memory>

/**
 * @class MotorController
 * @brief Motor controller node that interfaces with SparkMax motor controllers
 */
class MotorController : public rclcpp::Node
{
public:
  MotorController() : Node("motor_controller")
  {
    try
    {
      // Initialize all motor controllers
      left_wheel_motor_ = std::make_unique<SparkMax>("can0", 1);
      left_actuator_motor_ = std::make_unique<SparkMax>("can0", 2);
      right_wheel_motor_ = std::make_unique<SparkMax>("can0", 3);
      vibration_motor_ = std::make_unique<SparkMax>("can0", 4);
      right_actuator_motor_ = std::make_unique<SparkMax>("can0", 5);

      // Configure actuator motors
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

    // Subscribe to motor commands
    motor_cmd_sub_ = create_subscription<lunabot_msgs::msg::MotorCommands>(
      "/motor_commands", 10,
      std::bind(&MotorController::motor_cmd_callback, this, std::placeholders::_1));

    // Subscribe to robot disabled status
    robot_disabled_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/robot_disabled", 10,
      std::bind(&MotorController::robot_disabled_callback, this, std::placeholders::_1));

    // Heartbeat timer for actuator motor (20 Hz)
    if (motors_available_)
    {
      heartbeat_timer_ = create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&MotorController::heartbeat_callback, this));
    }

    LOGGER_INFO(get_logger(), "Motor controller node started");
  }

private:
  /**
   * @brief Send periodic heartbeat to actuator motor
   */
  void heartbeat_callback()
  {
    if (motors_available_ && !robot_disabled_)
    {
      left_actuator_motor_->Heartbeat();
    }
  }

  /**
   * @brief Apply motor commands from message
   */
  void motor_cmd_callback(const lunabot_msgs::msg::MotorCommands::SharedPtr msg)
  {
    if (!motors_available_) return;

    // Clamp all values to [-1, 1]
    auto clamp = [](float val) { return std::clamp(val, -1.0f, 1.0f); };

    left_wheel_motor_->SetDutyCycle(clamp(msg->left_wheel));
    right_wheel_motor_->SetDutyCycle(clamp(msg->right_wheel));
    left_actuator_motor_->SetDutyCycle(clamp(msg->left_actuator));
    right_actuator_motor_->SetDutyCycle(clamp(msg->right_actuator));
    vibration_motor_->SetDutyCycle(clamp(msg->vibration));
  }

  /**
   * @brief Handle robot disabled state updates
   */
  void robot_disabled_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    robot_disabled_ = msg->data;
  }

  rclcpp::Subscription<lunabot_msgs::msg::MotorCommands>::SharedPtr motor_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robot_disabled_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  std::unique_ptr<SparkMax> left_wheel_motor_;
  std::unique_ptr<SparkMax> left_actuator_motor_;
  std::unique_ptr<SparkMax> right_wheel_motor_;
  std::unique_ptr<SparkMax> vibration_motor_;
  std::unique_ptr<SparkMax> right_actuator_motor_;

  bool motors_available_ = false;
  bool robot_disabled_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorController>());
  rclcpp::shutdown();
  return 0;
}
