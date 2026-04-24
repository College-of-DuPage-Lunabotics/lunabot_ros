/**
 * @file depositing_server.cpp
 * @author Grayson Arendt
 * @date 02/22/2026
 */

#include "SparkMax.hpp"
#include "lunabot_logger/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "lunabot_msgs/action/depositing.hpp"
#include <std_msgs/msg/float64.hpp>

#include <chrono>
#include <memory>
#include <thread>

static constexpr double deposit_pos = 0.0;
static constexpr double travel_pos = 0.7854;
static constexpr double deposit_seconds = 7.0;

/**
 * @class DepositingServer
 * @brief Hardware depositing server that controls bucket actuators for depositing sequence.
 */
class DepositingServer : public rclcpp::Node
{
public:
  using Depositing = lunabot_msgs::action::Depositing;
  using GoalHandleDepositing = rclcpp_action::ServerGoalHandle<Depositing>;

  /**
   * @brief Constructor for the DepositingServer class.
   */
  DepositingServer()
  : Node("depositing_server"),
    right_actuator_motor_("can0", 5),
    left_actuator_motor_("can0", 2),
    vibration_motor_("can0", 4)
  {
    action_server_ = rclcpp_action::create_server<Depositing>(
      this, "depositing_action",
      [this](const auto &, const auto &) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const auto &) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const auto goal_handle) {
        std::thread{[this, goal_handle]() { execute(goal_handle); }}.detach();
      });

    encoder_position_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
      "bucket_angle", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
        current_encoder_position_ = msg->data;
      });

    LOGGER_SUCCESS(this->get_logger(), "Depositing server initialized");
  }

private:
  /**
   * @brief Lifts bucket to deposit position.
   * @return true if successful, false if cancelled
   */
  bool lift_bucket(const std::shared_ptr<GoalHandleDepositing> goal_handle)
  {
    LOGGER_ACTION(this->get_logger(), "Lifting bucket to deposit...");

    double target_position = deposit_pos;

    while (current_encoder_position_ > target_position)
    {
      if (goal_handle->is_canceling())
      {
        left_actuator_motor_.SetDutyCycle(0.0);
        right_actuator_motor_.SetDutyCycle(0.0);
        return false;
      }

      left_actuator_motor_.Heartbeat();
      right_actuator_motor_.Heartbeat();

      left_actuator_motor_.SetDutyCycle(-1.0);
      right_actuator_motor_.SetDutyCycle(-1.0);

      LOGGER_INFO(
        this->get_logger(), "Lifting... Current position: %.2f, Target: %.2f",
        current_encoder_position_, target_position);
    }

    left_actuator_motor_.SetDutyCycle(0.0);
    right_actuator_motor_.SetDutyCycle(0.0);

    LOGGER_ACTION(this->get_logger(), "Vibrating bucket for %.2f seconds...", deposit_seconds);

    auto start = std::chrono::steady_clock::now();

    bool vibration_on = true;

    while (vibration_on)
    {
      if (goal_handle->is_canceling())
      {
        vibration_motor_.SetDutyCycle(0.0);
        return false;
      }

      vibration_motor_.Heartbeat();
      vibration_motor_.SetDutyCycle(1.0);

      auto end = std::chrono::steady_clock::now();

      std::chrono::duration<double> elapsed_seconds = end - start;

      if (elapsed_seconds.count() >= deposit_seconds)
      {
        vibration_on = false;
        vibration_motor_.Heartbeat();
        vibration_motor_.SetDutyCycle(0.0);
      }
    }

    return true;
  }

  /**
   * @brief Lowers bucket back to travel position.
   * @return true if successful, false if cancelled
   */
  bool lower_bucket(const std::shared_ptr<GoalHandleDepositing> goal_handle)
  {
    LOGGER_ACTION(this->get_logger(), "Lowering bucket to travel position...");

    double target_position = travel_pos;

    while (current_encoder_position_ < target_position)
    {
      if (goal_handle->is_canceling())
      {
        left_actuator_motor_.SetDutyCycle(0.0);
        right_actuator_motor_.SetDutyCycle(0.0);
        return false;
      }

      left_actuator_motor_.Heartbeat();
      left_actuator_motor_.SetDutyCycle(1.0);
      right_actuator_motor_.SetDutyCycle(1.0);
    }

    left_actuator_motor_.SetDutyCycle(0.0);
    right_actuator_motor_.SetDutyCycle(0.0);

    return true;
  }

  /**
   * @brief Executes the depositing action sequence.
   * @param goal_handle Handle for the action goal.
   */
  void execute(const std::shared_ptr<GoalHandleDepositing> goal_handle)
  {
    if (goal_active_)
    {
      LOGGER_WARN(this->get_logger(), "Depositing already in progress");
      auto result = std::make_shared<Depositing::Result>();
      result->success = false;
      result->message = "Depositing already in progress";
      goal_handle->abort(result);
      return;
    }

    goal_active_ = true;
    LOGGER_SUCCESS(this->get_logger(), "Starting depositing sequence");

    auto feedback = std::make_shared<Depositing::Feedback>();
    auto result = std::make_shared<Depositing::Result>();

    try
    {
      feedback->feedback_message = "Lifting bucket to deposit position";
      goal_handle->publish_feedback(feedback);
      if (!lift_bucket(goal_handle))
      {
        left_actuator_motor_.SetDutyCycle(0.0);
        right_actuator_motor_.SetDutyCycle(0.0);
        vibration_motor_.SetDutyCycle(0.0);
        result->success = false;
        result->message = "Depositing cancelled";
        goal_handle->canceled(result);
        LOGGER_WARN(this->get_logger(), "Depositing cancelled");
        goal_active_ = false;
        return;
      }

      feedback->feedback_message = "Returning bucket to home position";
      goal_handle->publish_feedback(feedback);
      if (!lower_bucket(goal_handle))
      {
        left_actuator_motor_.SetDutyCycle(0.0);
        right_actuator_motor_.SetDutyCycle(0.0);
        vibration_motor_.SetDutyCycle(0.0);
        result->success = false;
        result->message = "Depositing cancelled";
        goal_handle->canceled(result);
        LOGGER_WARN(this->get_logger(), "Depositing cancelled");
        goal_active_ = false;
        return;
      }

      result->success = true;
      result->message = "Depositing completed successfully";
      goal_handle->succeed(result);
      LOGGER_SUCCESS(this->get_logger(), "Depositing completed successfully");
    } catch (const std::exception & e)
    {
      left_actuator_motor_.SetDutyCycle(0.0);
      right_actuator_motor_.SetDutyCycle(0.0);
      vibration_motor_.SetDutyCycle(0.0);
      result->success = false;
      result->message = std::string("Depositing failed: ") + e.what();
      goal_handle->abort(result);
      LOGGER_FAILURE(this->get_logger(), "Depositing failed: %s", e.what());
    }

    goal_active_ = false;
  }

  rclcpp_action::Server<Depositing>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr encoder_position_subscriber_;

  double current_encoder_position_ = 0.0;
  SparkMax left_actuator_motor_;
  SparkMax right_actuator_motor_;
  SparkMax vibration_motor_;
  bool goal_active_ = false;
};

/**
 * @brief Main function.
 * Initializes and runs the DepositingServer node.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepositingServer>());
  rclcpp::shutdown();
  return 0;
}
