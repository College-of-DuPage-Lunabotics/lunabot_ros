/**
 * @file depositing_server.cpp
 * @author Grayson Arendt
 * @date 02/22/2026
 */

#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "lunabot_msgs/action/depositing.hpp"
#include "lunabot_logger/logger.hpp"

#include "SparkMax.hpp"
#include <std_msgs/msg/float64.hpp>

#define DEPOSIT_POS 9.0
#define TRAVEL_POS 2.0

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
    : Node("depositing_server")
    , goal_active_(false)
    , home_offset_(0.0)
    , right_actuator_motor_("can0", 1)
    , left_actuator_motor_("can0", 2)
    , vibration_motor_("can0", 5)
  {
    action_server_ = rclcpp_action::create_server<Depositing>(
        this, "depositing_action",
        [this](const auto&, const auto&) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [this](const auto&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const auto goal_handle) { std::thread{ [this, goal_handle]() { execute(goal_handle); } }.detach(); });

    home_offset_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
        "actuator_home_offset", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          home_offset_ = msg->data;
          LOGGER_ACTION(this->get_logger(), "Home offset updated: %.2f", home_offset_);
        });

    LOGGER_SUCCESS(this->get_logger(), "Depositing server initialized");
  }

private:
  /**
   * @brief Gets the position of the left bucket actuator (right encoder not working).
   * @return Left encoder position in radians.
   */
  double get_actuator_position()
  {
    left_actuator_motor_.Heartbeat();
    right_actuator_motor_.Heartbeat();
    double left_pos = left_actuator_motor_.GetPosition();
    // Subtract home offset to get position relative to home
    return left_pos - home_offset_;
  }

  /**
   * @brief Lifts bucket to deposit position.
   */
  void lift_bucket()
  {
    LOGGER_ACTION(this->get_logger(), "Lifting bucket to deposit...");

    // Absolute target from zero
    double target_position = DEPOSIT_POS;

    // Lift bucket without vibration first
    while (get_actuator_position() < target_position)
    {
      left_actuator_motor_.Heartbeat();
      right_actuator_motor_.Heartbeat();

      left_actuator_motor_.SetDutyCycle(-1.0);
      right_actuator_motor_.SetDutyCycle(-1.0);

      LOGGER_INFO(this->get_logger(), "Lifting... Current position: %.2f, Target: %.2f", get_actuator_position(), target_position);
    }

    // Stop the actuators
    left_actuator_motor_.SetDutyCycle(0.0);
    right_actuator_motor_.SetDutyCycle(0.0);

    // Vibrate at deposit position for a couple seconds
    LOGGER_ACTION(this->get_logger(), "Vibrating bucket for 5 seconds...");
    vibration_motor_.Heartbeat();
    vibration_motor_.SetDutyCycle(1.0);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    vibration_motor_.SetDutyCycle(0.0);
  }

  /**
   * @brief Lowers bucket back to home position.
   */
  void lower_bucket()
  {
    LOGGER_ACTION(this->get_logger(), "Lowering bucket to home...");

    // Absolute target from zero
    double target_position = TRAVEL_POS;

    while (get_actuator_position() > target_position)
    {
      left_actuator_motor_.Heartbeat();
      right_actuator_motor_.Heartbeat();

      left_actuator_motor_.SetDutyCycle(1.0);
      right_actuator_motor_.SetDutyCycle(1.0);
    }

    // Stop motors
    left_actuator_motor_.SetDutyCycle(0.0);
    right_actuator_motor_.SetDutyCycle(0.0);
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
      lift_bucket();

      feedback->feedback_message = "Returning bucket to home position";
      goal_handle->publish_feedback(feedback);
      lower_bucket();

      result->success = true;
      result->message = "Depositing completed successfully";
      goal_handle->succeed(result);
      LOGGER_SUCCESS(this->get_logger(), "Depositing completed successfully");
    }
    catch (const std::exception& e)
    {
      result->success = false;
      result->message = std::string("Depositing failed: ") + e.what();
      goal_handle->abort(result);
      LOGGER_FAILURE(this->get_logger(), "Depositing failed: %s", e.what());
    }

    goal_active_ = false;
  }

  rclcpp_action::Server<Depositing>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr home_offset_subscriber_;

  double home_offset_;
  SparkMax left_actuator_motor_;
  SparkMax right_actuator_motor_;
  SparkMax vibration_motor_;
  bool goal_active_;
};

/**
 * @brief Main function.
 * Initializes and runs the DepositingServer node.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepositingServer>());
  rclcpp::shutdown();
  return 0;
}
