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
#include "SparkMax.hpp"

// ANSI color codes
#define RESET "\033[0m"
#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define CYAN "\033[1;36m"

#define HOMING_SPEED 0.5
#define POSITION_THRESHOLD 10.0
#define LIFT_TICKS 1000.0

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
    : Node("homing_server")
    , goal_active_(false)
    , left_actuator_motor_("can0", 2)
    , right_actuator_motor_("can0", 1)
  {
    action_server_ = rclcpp_action::create_server<Homing>(
        this, "homing_action",
        [this](const auto&, const auto&) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [this](const auto&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const auto goal_handle) { std::thread{ [this, goal_handle]() { execute(goal_handle); } }.detach(); });

    left_actuator_motor_.SetSensorType(SensorType::kEncoder);
    right_actuator_motor_.SetSensorType(SensorType::kEncoder);
    left_actuator_motor_.BurnFlash();
    right_actuator_motor_.BurnFlash();

    // Declare parameter for home offset
    this->declare_parameter<double>("actuator_home_offset", 0.0);

    RCLCPP_INFO(this->get_logger(), GREEN "HOMING SERVER INITIALIZED" RESET);
  }

private:
  /**
   * @brief Gets the average position of both bucket actuators.
   * @return Average encoder position in ticks.
   */
  double get_actuator_position()
  {
    left_actuator_motor_.Heartbeat();
    right_actuator_motor_.Heartbeat();
    double left_pos = left_actuator_motor_.GetPosition();
    double right_pos = right_actuator_motor_.GetPosition();
    return (left_pos + right_pos) / 2.0;
  }

  /**
   * @brief Extends actuators to hard stop and sets zero position.
   */
  void home_actuators()
  {
    RCLCPP_INFO(this->get_logger(), CYAN "EXTENDING ACTUATORS TO HARD STOP..." RESET);

    double previous_position = get_actuator_position();
    int stall_count = 0;

    // Move actuators until they stop (hit hard limit)
    while (stall_count < 20)
    {
      left_actuator_motor_.Heartbeat();
      right_actuator_motor_.Heartbeat();

      left_actuator_motor_.SetDutyCycle(HOMING_SPEED);
      right_actuator_motor_.SetDutyCycle(HOMING_SPEED);

      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      double current_position = get_actuator_position();
      
      // Check if position has changed significantly
      if (std::abs(current_position - previous_position) < POSITION_THRESHOLD)
      {
        stall_count++;
      }
      else
      {
        stall_count = 0;
      }
      
      previous_position = current_position;
    }

    // Stop motors
    left_actuator_motor_.SetDutyCycle(0.0);
    right_actuator_motor_.SetDutyCycle(0.0);

    RCLCPP_INFO(this->get_logger(), GREEN "HARD STOP REACHED" RESET);

    // Record the current position as the home offset
    double home_offset = get_actuator_position();
    this->set_parameter(rclcpp::Parameter("actuator_home_offset", home_offset));

    RCLCPP_INFO(this->get_logger(), GREEN "ACTUATOR HOME OFFSET SET: %.2f ticks" RESET, home_offset);
  }

  /**
   * @brief Retracts actuators to neutral travel position.
   */
  void return_to_neutral()
  {
    RCLCPP_INFO(this->get_logger(), CYAN "RETURNING TO NEUTRAL POSITION..." RESET);

    double initial_position = get_actuator_position();
    double target_position = initial_position - LIFT_TICKS;

    while (get_actuator_position() > target_position)
    {
      left_actuator_motor_.Heartbeat();
      right_actuator_motor_.Heartbeat();

      left_actuator_motor_.SetDutyCycle(-HOMING_SPEED);
      right_actuator_motor_.SetDutyCycle(-HOMING_SPEED);

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Stop motors
    left_actuator_motor_.SetDutyCycle(0.0);
    right_actuator_motor_.SetDutyCycle(0.0);

    RCLCPP_INFO(this->get_logger(), GREEN "NEUTRAL POSITION REACHED" RESET);
  }

  /**
   * @brief Executes the homing action sequence.
   * @param goal_handle Handle for the action goal.
   */
  void execute(const std::shared_ptr<GoalHandleHoming> goal_handle)
  {
    if (goal_active_)
    {
      RCLCPP_WARN(this->get_logger(), YELLOW "HOMING ALREADY IN PROGRESS" RESET);
      auto result = std::make_shared<Homing::Result>();
      result->success = false;
      result->message = "Homing already in progress";
      goal_handle->abort(result);
      return;
    }

    goal_active_ = true;
    RCLCPP_INFO(this->get_logger(), GREEN "STARTING HOMING SEQUENCE" RESET);

    auto feedback = std::make_shared<Homing::Feedback>();
    auto result = std::make_shared<Homing::Result>();

    try
    {
      feedback->feedback_message = "Extending actuators to home position";
      goal_handle->publish_feedback(feedback);
      home_actuators();

      feedback->feedback_message = "Returning to neutral position";
      goal_handle->publish_feedback(feedback);
      return_to_neutral();

      result->success = true;
      result->message = "Homing completed successfully";
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), GREEN "HOMING COMPLETED SUCCESSFULLY" RESET);
    }
    catch (const std::exception& e)
    {
      result->success = false;
      result->message = std::string("Homing failed: ") + e.what();
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), RED "HOMING FAILED: %s" RESET, e.what());
    }

    goal_active_ = false;
  }

  rclcpp_action::Server<Homing>::SharedPtr action_server_;

  SparkMax left_actuator_motor_;
  SparkMax right_actuator_motor_;
  bool goal_active_;
};

/**
 * @brief Main function.
 * Initializes and runs the HomingServer node.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HomingServer>());
  rclcpp::shutdown();
  return 0;
}
