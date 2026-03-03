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

#define LIFT_TICKS 1000.0

/**
 * @class DepositingServer
 * @brief Hardware depositing server that controls bucket actuators for depositing operations.
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
    , left_actuator_motor_("can0", 2)
    , right_actuator_motor_("can0", 1)
    , vibration_motor_("can0", 5)
  {
    action_server_ = rclcpp_action::create_server<Depositing>(
        this, "depositing_action",
        [this](const auto&, const auto&) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [this](const auto&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const auto goal_handle) { std::thread{ [this, goal_handle]() { execute(goal_handle); } }.detach(); });

    left_actuator_motor_.SetSensorType(SensorType::kEncoder);
    right_actuator_motor_.SetSensorType(SensorType::kEncoder);
    left_actuator_motor_.BurnFlash();
    right_actuator_motor_.BurnFlash();

    LOGGER_SUCCESS(this->get_logger(), "Depositing server initialized");
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
    // Subtract home offset to get position relative to home
    return (left_pos + right_pos) / 2.0 - home_offset_;
  }

  /**
   * @brief Updates the home offset from homing server parameter.
   */
  void update_home_offset()
  {
    auto homing_node = std::make_shared<rclcpp::Node>("homing_offset_reader");
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(homing_node, "/homing_server");
    if (param_client->wait_for_service(std::chrono::seconds(1)))
    {
      auto params = param_client->get_parameters({ "actuator_home_offset" });
      if (!params.empty())
      {
        home_offset_ = params[0].as_double();
        LOGGER_ACTION(this->get_logger(), "Home offset updated: %.2f ticks", home_offset_);
      }
    }
  }

  /**
   * @brief Lifts bucket to dump position.
   */
  void lift_bucket()
  {
    LOGGER_ACTION(this->get_logger(), "Lifting bucket to dump...");

    double initial_position = get_actuator_position();
    double target_position = initial_position + LIFT_TICKS;

    // Turn on vibration to help with depositing
    vibration_motor_.Heartbeat();
    vibration_motor_.SetDutyCycle(1.0);

    while (get_actuator_position() < target_position)
    {
      left_actuator_motor_.Heartbeat();
      right_actuator_motor_.Heartbeat();
      vibration_motor_.Heartbeat();

      left_actuator_motor_.SetDutyCycle(1.0);
      right_actuator_motor_.SetDutyCycle(1.0);

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    left_actuator_motor_.SetDutyCycle(0.0);
    right_actuator_motor_.SetDutyCycle(0.0);
    vibration_motor_.SetDutyCycle(0.0);

    LOGGER_ACTION(this->get_logger(), "Waiting for material to dump...");
    std::this_thread::sleep_for(std::chrono::seconds(3));
  }

  /**
   * @brief Lowers bucket back to home position.
   */
  void lower_bucket()
  {
    LOGGER_ACTION(this->get_logger(), "Lowering bucket to home...");

    double initial_position = get_actuator_position();
    double target_position = initial_position - LIFT_TICKS;

    while (get_actuator_position() > target_position)
    {
      left_actuator_motor_.Heartbeat();
      right_actuator_motor_.Heartbeat();

      left_actuator_motor_.SetDutyCycle(-1.0);
      right_actuator_motor_.SetDutyCycle(-1.0);

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

    // Update home offset before starting
    update_home_offset();

    auto feedback = std::make_shared<Depositing::Feedback>();
    auto result = std::make_shared<Depositing::Result>();

    try
    {
      feedback->feedback_message = "Lifting bucket to dump position";
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
