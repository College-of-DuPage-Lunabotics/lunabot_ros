/**
 * @file depositing_server_sim.cpp
 * @author Grayson Arendt
 * @date 02/22/2026
 */

#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "lunabot_msgs/action/depositing.hpp"
#include "lunabot_logger/logger.hpp"

#define DEPOSIT_POSITION -1.5
#define TRAVEL_POSITION -0.4

/**
 * @class DepositingServerSim
 * @brief Simulation-compatible depositing server that controls bucket joint position.
 */
class DepositingServerSim : public rclcpp::Node
{
public:
  using Depositing = lunabot_msgs::action::Depositing;
  using GoalHandleDepositing = rclcpp_action::ServerGoalHandle<Depositing>;

  /**
   * @brief Constructor for the DepositingServerSim class.
   */
  DepositingServerSim() : Node("depositing_server"), goal_active_(false)
  {
    action_server_ = rclcpp_action::create_server<Depositing>(
        this, "depositing_action",
        [this](const auto&, const auto&) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [this](const auto&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const auto goal_handle) { std::thread{ [this, goal_handle]() { execute(goal_handle); } }.detach(); });

    bucket_position_pub_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);

    LOGGER_SUCCESS(this->get_logger(), "Depositing server (sim) initialized");
  }

private:
  /**
   * @brief Sets the bucket joint position.
   * @param position Target position in radians.
   */
  void set_bucket_position(double position)
  {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.push_back(position);
    bucket_position_pub_->publish(msg);
    LOGGER_ACTION(this->get_logger(), "Setting bucket position to %.2f rad", position);
  }

  /**
   * @brief Lifts bucket to deposit position and returns to travel position.
   */
  void lift_and_deposit()
  {
    // Lift bucket to deposit position
    LOGGER_ACTION(this->get_logger(), "Lifting bucket to deposit (vibration on)...");
    set_bucket_position(DEPOSIT_POSITION);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    LOGGER_SUCCESS(this->get_logger(), "Deposit complete");

    // Return bucket to travel position
    LOGGER_ACTION(this->get_logger(), "Resetting bucket to travel position...");
    set_bucket_position(TRAVEL_POSITION);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    LOGGER_SUCCESS(this->get_logger(), "Bucket reset complete");
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
      feedback->feedback_message = "Depositing and resetting bucket";
      goal_handle->publish_feedback(feedback);
      lift_and_deposit();

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
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr bucket_position_pub_;
  bool goal_active_;
};

/**
 * @brief Main function.
 * Initializes and runs the DepositingServerSim node.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepositingServerSim>());
  rclcpp::shutdown();
  return 0;
}
