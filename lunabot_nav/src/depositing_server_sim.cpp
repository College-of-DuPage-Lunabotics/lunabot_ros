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

// ANSI color codes
#define RESET "\033[0m"
#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define CYAN "\033[1;36m"

#define DUMP_POSITION -1.5
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
  DepositingServerSim()
    : Node("depositing_server")
    , goal_active_(false)
  {
    action_server_ = rclcpp_action::create_server<Depositing>(
        this, "depositing_action",
        [this](const auto&, const auto&) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [this](const auto&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const auto goal_handle) { std::thread{ [this, goal_handle]() { execute(goal_handle); } }.detach(); });

    bucket_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/position_controller/commands", 10);

    RCLCPP_INFO(this->get_logger(), GREEN "DEPOSITING SERVER (SIM) INITIALIZED" RESET);
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
    RCLCPP_INFO(this->get_logger(), CYAN "Setting bucket position to %.2f rad" RESET, position);
  }

  /**
   * @brief Lifts bucket to dump position and returns to travel position.
   */
  void lift_and_dump()
  {
    // Lift bucket to dump position
    RCLCPP_INFO(this->get_logger(), CYAN "LIFTING BUCKET TO DUMP (vibration ON)..." RESET);
    set_bucket_position(DUMP_POSITION);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(this->get_logger(), GREEN "DUMP COMPLETE" RESET);

    // Return bucket to travel position
    RCLCPP_INFO(this->get_logger(), CYAN "RESETTING BUCKET TO TRAVEL POSITION..." RESET);
    set_bucket_position(TRAVEL_POSITION);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(this->get_logger(), GREEN "BUCKET RESET COMPLETE" RESET);
  }

  /**
   * @brief Executes the depositing action sequence.
   * @param goal_handle Handle for the action goal.
   */
  void execute(const std::shared_ptr<GoalHandleDepositing> goal_handle)
  {
    if (goal_active_)
    {
      RCLCPP_WARN(this->get_logger(), YELLOW "DEPOSITING ALREADY IN PROGRESS" RESET);
      auto result = std::make_shared<Depositing::Result>();
      result->success = false;
      result->message = "Depositing already in progress";
      goal_handle->abort(result);
      return;
    }

    goal_active_ = true;
    RCLCPP_INFO(this->get_logger(), GREEN "STARTING DEPOSITING SEQUENCE" RESET);

    auto feedback = std::make_shared<Depositing::Feedback>();
    auto result = std::make_shared<Depositing::Result>();

    try
    {
      feedback->feedback_message = "Depositing and resetting bucket";
      goal_handle->publish_feedback(feedback);
      lift_and_dump();

      result->success = true;
      result->message = "Depositing completed successfully";
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), GREEN "DEPOSITING COMPLETED SUCCESSFULLY" RESET);
    }
    catch (const std::exception& e)
    {
      result->success = false;
      result->message = std::string("Depositing failed: ") + e.what();
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), RED "DEPOSITING FAILED: %s" RESET, e.what());
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
