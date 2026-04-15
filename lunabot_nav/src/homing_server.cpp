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
#define POSITION_THRESHOLD 0.02
#define STALL_DURATION_SEC 3
#define TRAVEL_POS 6.0

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
    : Node("homing_server"), goal_active_(false), right_actuator_motor_("can0", 5), left_actuator_motor_("can0", 2)
  {
    action_server_ = rclcpp_action::create_server<Homing>(
        this, "homing_action",
        [this](const auto&, const auto&) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [this](const auto&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const auto goal_handle) { std::thread{ [this, goal_handle]() { execute(goal_handle); } }.detach(); });

    home_offset_publisher_ = this->create_publisher<std_msgs::msg::Float64>("actuator_home_offset", 10);

    this->declare_parameter<double>("actuator_home_offset", 0.0);

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
    double left_pos = left_actuator_motor_.GetPosition();
    return left_pos;
  }

  /**
   * @brief Extends actuators to hard stop and sets zero position.
   * @return true if successful, false if cancelled
   */
  bool home_actuators(const std::shared_ptr<GoalHandleHoming> goal_handle)
  {
    LOGGER_ACTION(this->get_logger(), "Extending actuators to hard stop...");

    double previous_position = get_actuator_position();
    auto last_check_time = std::chrono::steady_clock::now();
    auto stall_start_time = std::chrono::steady_clock::now();
    bool is_stalled = false;

    while (true)
    {
      if (goal_handle->is_canceling())
      {
        left_actuator_motor_.SetDutyCycle(0.0);
        right_actuator_motor_.SetDutyCycle(0.0);
        return false;
      }

      left_actuator_motor_.Heartbeat();

      left_actuator_motor_.SetDutyCycle(-HOMING_SPEED);
      right_actuator_motor_.SetDutyCycle(-HOMING_SPEED);

      auto now = std::chrono::steady_clock::now();
      auto elapsed_since_check = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_check_time);
      
      if (elapsed_since_check.count() >= 100) // Check every 100ms if stalled
      {
        double current_position = get_actuator_position();
        double position_change = std::abs(current_position - previous_position);

        if (position_change < POSITION_THRESHOLD)
        {
          if (!is_stalled)
          {
            stall_start_time = now;
            is_stalled = true;
          }
          else
          {
            auto stall_duration = std::chrono::duration_cast<std::chrono::seconds>(now - stall_start_time);
            if (stall_duration.count() >= STALL_DURATION_SEC)
            {
              break;
            }
          }
        }
        else
        {
          is_stalled = false;
        }

        previous_position = current_position;
        last_check_time = now;
      }
    }

    left_actuator_motor_.SetDutyCycle(0.0);
    right_actuator_motor_.SetDutyCycle(0.0);

    LOGGER_SUCCESS(this->get_logger(), "Hard stop reached");

    double home_offset = get_actuator_position();

    auto msg = std_msgs::msg::Float64();
    msg.data = home_offset;
    home_offset_publisher_->publish(msg);

    LOGGER_SUCCESS(this->get_logger(), "Actuator home offset set: %.2f", home_offset);
    return true;
  }

  /**
   * @brief Retracts actuators to neutral travel position.
   * @return true if successful, false if cancelled
   */
  bool return_to_neutral(const std::shared_ptr<GoalHandleHoming> goal_handle)
  {
    LOGGER_ACTION(this->get_logger(), "Returning to neutral position...");

    double initial_position = get_actuator_position();
    double target_position = initial_position - TRAVEL_POS;

    while (get_actuator_position() > target_position)
    {
      if (goal_handle->is_canceling())
      {
        left_actuator_motor_.SetDutyCycle(0.0);
        right_actuator_motor_.SetDutyCycle(0.0);
        return false;
      }

      left_actuator_motor_.Heartbeat();

      left_actuator_motor_.SetDutyCycle(HOMING_SPEED);
      right_actuator_motor_.SetDutyCycle(HOMING_SPEED);
    }

    left_actuator_motor_.SetDutyCycle(0.0);
    right_actuator_motor_.SetDutyCycle(0.0);

    LOGGER_SUCCESS(this->get_logger(), "Neutral position reached");
    return true;
  }

  /**
   * @brief Executes the homing action sequence.
   * @param goal_handle Handle for the action goal.
   */
  void execute(const std::shared_ptr<GoalHandleHoming> goal_handle)
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
      if (!home_actuators(goal_handle))
      {
        left_actuator_motor_.SetDutyCycle(0.0);
        right_actuator_motor_.SetDutyCycle(0.0);
        result->success = false;
        result->message = "Homing cancelled";
        goal_handle->canceled(result);
        LOGGER_WARN(this->get_logger(), "Homing cancelled");
        goal_active_ = false;
        return;
      }

      feedback->feedback_message = "Returning to neutral position";
      goal_handle->publish_feedback(feedback);
      if (!return_to_neutral(goal_handle))
      {
        left_actuator_motor_.SetDutyCycle(0.0);
        right_actuator_motor_.SetDutyCycle(0.0);
        result->success = false;
        result->message = "Homing cancelled";
        goal_handle->canceled(result);
        LOGGER_WARN(this->get_logger(), "Homing cancelled");
        goal_active_ = false;
        return;
      }

      result->success = true;
      result->message = "Homing completed successfully";
      goal_handle->succeed(result);
      LOGGER_SUCCESS(this->get_logger(), "Homing completed successfully");
    }
    catch (const std::exception& e)
    {
      left_actuator_motor_.SetDutyCycle(0.0);
      right_actuator_motor_.SetDutyCycle(0.0);
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
