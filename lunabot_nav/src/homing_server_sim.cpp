/**
 * @file homing_server_sim.cpp
 * @author Grayson Arendt
 * @date 02/22/2026
 */

#include <chrono>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "lunabot_msgs/action/homing.hpp"

// ANSI color codes
#define RESET "\033[0m"
#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define CYAN "\033[1;36m"

#define HOME_POSITION -1.5
#define TRAVEL_POSITION -0.4

/**
 * @class HomingServerSim
 * @brief Simulation-compatible homing server that sets bucket to known home position.
 */
class HomingServerSim : public rclcpp::Node
{
public:
  using Homing = lunabot_msgs::action::Homing;
  using GoalHandleHoming = rclcpp_action::ServerGoalHandle<Homing>;

  /**
   * @brief Constructor for the HomingServerSim class.
   */
  HomingServerSim()
    : Node("homing_server")
    , goal_active_(false)
  {
    action_server_ = rclcpp_action::create_server<Homing>(
        this, "homing_action",
        [this](const auto&, const auto&) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [this](const auto&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const auto goal_handle) { std::thread{ [this, goal_handle]() { execute(goal_handle); } }.detach(); });

    bucket_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/position_controller/commands", 10);

    RCLCPP_INFO(this->get_logger(), GREEN "HOMING SERVER (SIM) INITIALIZED" RESET);
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
   * @brief Moves bucket to home position and back to travel position.
   */
  void home_bucket()
  {
    // Move bucket to fully extended home position
    RCLCPP_INFO(this->get_logger(), CYAN "MOVING BUCKET TO HOME POSITION..." RESET);
    set_bucket_position(HOME_POSITION);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    RCLCPP_INFO(this->get_logger(), GREEN "HOME POSITION REACHED" RESET);

    // Return to neutral travel position
    RCLCPP_INFO(this->get_logger(), CYAN "RETURNING TO TRAVEL POSITION..." RESET);
    set_bucket_position(TRAVEL_POSITION);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(this->get_logger(), GREEN "TRAVEL POSITION REACHED" RESET);
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
      feedback->feedback_message = "Moving to home position";
      goal_handle->publish_feedback(feedback);
      home_bucket();

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
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr bucket_position_pub_;
  bool goal_active_;
};

/**
 * @brief Main function.
 * Initializes and runs the HomingServerSim node.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HomingServerSim>());
  rclcpp::shutdown();
  return 0;
}
