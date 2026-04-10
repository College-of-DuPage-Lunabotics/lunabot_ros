/**
 * @file homing_server.cpp
 * @author Grayson Arendt
 * @date 02/22/2026
 */

#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "lunabot_msgs/action/homing.hpp"
#include "lunabot_logger/logger.hpp"
#include "SparkMax.hpp"
#include <std_msgs/msg/float64.hpp>

#define HOMING_SPEED 1.0
#define HOMING_TIME_SEC 10.0
#define TRAVEL_POS 2.0

/**
 * @class HomingServer
 * @brief Hardware homing server that establishes actuator zero position at full extension.
 */
class HomingServer : public rclcpp::Node
{
public:
  using Homing = lunabot_msgs::action::Homing;
  using GoalHandleHoming = rclcpp_action::ServerGoalHandle<Homing>;

  /**
   * @brief Constructor for the HomingServer class.
   */
  HomingServer()
      : Node("homing_server"), goal_active_(false), right_actuator_motor_("can0", 1), left_actuator_motor_("can0", 2)
  {
    action_server_ = rclcpp_action::create_server<Homing>(
        this, "homing_action",
        [this](const auto &, const auto &)
        { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [this](const auto &)
        { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const auto goal_handle)
        { std::thread{[this, goal_handle]()
                      { execute(goal_handle); }}
              .detach(); });

    home_offset_publisher_ = this->create_publisher<std_msgs::msg::Float64>("actuator_home_offset", 10);

    LOGGER_SUCCESS(this->get_logger(), "Homing server initialized");
  }

private:
  /**
   * @brief Gets the position of the left bucket actuator.
   * @return Left encoder position in radians.
   */
  double get_actuator_position()
  {
    left_actuator_motor_.Heartbeat();
    right_actuator_motor_.Heartbeat();
    double left_pos = left_actuator_motor_.GetPosition();
    return left_pos;
  }

  /**
   * @brief Extends actuators to hard stop and zeros position.
   */
  void home_actuators()
  {
    LOGGER_ACTION(this->get_logger(), "Extending actuators to hard stop...");

    // Move bucket up for set amount of time, it will hard stop
    auto start_time = std::chrono::steady_clock::now();
    auto homing_duration = std::chrono::duration<double>(HOMING_TIME_SEC);
    while (std::chrono::steady_clock::now() - start_time < homing_duration)
    {
      left_actuator_motor_.Heartbeat();
      right_actuator_motor_.Heartbeat();
      left_actuator_motor_.SetDutyCycle(-HOMING_SPEED);
      right_actuator_motor_.SetDutyCycle(-HOMING_SPEED);
    }

    // Stop motors
    left_actuator_motor_.Heartbeat();
    right_actuator_motor_.Heartbeat();
    left_actuator_motor_.SetDutyCycle(0.0);
    right_actuator_motor_.SetDutyCycle(0.0);

    LOGGER_SUCCESS(this->get_logger(), "Hard stop reached");

    // Zero the encoders at this position
    left_actuator_motor_.Heartbeat();
    right_actuator_motor_.Heartbeat();
    left_actuator_motor_.SetPosition(0.0);
    right_actuator_motor_.SetPosition(0.0);
  }

  /**
   * @brief Returns actuators to travel position after homing.
   */
  void return_to_travel()
  {
    LOGGER_ACTION(this->get_logger(), "Returning to travel position...");

    // Move down until reaching travel position
    while (get_actuator_position() > TRAVEL_POS)
    {
      left_actuator_motor_.Heartbeat();
      left_actuator_motor_.SetDutyCycle(HOMING_SPEED);
      right_actuator_motor_.SetDutyCycle(HOMING_SPEED);
    }

    // Stop motors
    left_actuator_motor_.Heartbeat();
    left_actuator_motor_.SetDutyCycle(0.0);
    right_actuator_motor_.SetDutyCycle(0.0);

    LOGGER_SUCCESS(this->get_logger(), "Travel position reached: %.2f", get_actuator_position());
  }
}

/**
 * @brief Executes the homing action sequence.
 * @param goal_handle Handle for the action goal.
 */
void
execute(const std::shared_ptr<GoalHandleHoming> goal_handle)
{
  if (goal_active_)
  {
    LOGGER_WARN(this->get_logger(), "Homing already in progress");
    auto result = std::make_shared<Homing::Result>();
    result->success = false;
    result->message = "Homing already in progress";
    goal_handle->abort(result);
    return;
  }

  goal_active_ = true;
  LOGGER_SUCCESS(this->get_logger(), "Starting homing sequence");

  auto feedback = std::make_shared<Homing::Feedback>();
  auto result = std::make_shared<Homing::Result>();

  try
  {
    feedback->feedback_message = "Extending actuators to home position";
    goal_handle->publish_feedback(feedback);
    home_actuators();

    feedback->feedback_message = "Returning to neutral position";
    goal_handle->publish_feedback(feedback);
    return_to_travel();

    result->success = true;
    result->message = "Homing completed successfully";
    goal_handle->succeed(result);
    LOGGER_SUCCESS(this->get_logger(), "Homing completed successfully");
  }
  catch (const std::exception &e)
  {
    result->success = false;
    result->message = std::string("Homing failed: ") + e.what();
    goal_handle->abort(result);
    LOGGER_FAILURE(this->get_logger(), "Homing failed: %s", e.what());
  }

  goal_active_ = false;
}

rclcpp_action::Server<Homing>::SharedPtr action_server_;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr home_offset_publisher_;

SparkMax left_actuator_motor_;
SparkMax right_actuator_motor_;
bool goal_active_;

/**
 * @brief Main function.
 * Initializes and runs the HomingServer node.
 */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HomingServer>());
  rclcpp::shutdown();
  return 0;
}
