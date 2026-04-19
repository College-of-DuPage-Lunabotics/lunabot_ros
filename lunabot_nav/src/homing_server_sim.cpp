/**
 * @file homing_server_sim.cpp
 * @author Grayson Arendt
 * @date 02/22/2026
 */

#include "lunabot_logger/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "lunabot_msgs/action/homing.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <chrono>
#include <memory>
#include <thread>

static constexpr double home_position = -1.5;
static constexpr double travel_position = -0.4;

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
  HomingServerSim() : Node("homing_server")
  {
    action_server_ = rclcpp_action::create_server<Homing>(
      this, "homing_action",
      [this](const auto &, const auto &) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const auto &) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const auto goal_handle) {
        std::thread{[this, goal_handle]() { execute(goal_handle); }}.detach();
      });

    bucket_position_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);

    LOGGER_SUCCESS(this->get_logger(), "Homing server (sim) initialized");
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
   * @brief Moves bucket to home position and back to travel position.
   */
  void home_bucket()
  {
    // Move bucket to fully extended home position
    LOGGER_ACTION(this->get_logger(), "Moving bucket to home position...");
    set_bucket_position(home_position);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    LOGGER_SUCCESS(this->get_logger(), "Home position reached");

    // Return to neutral travel position
    LOGGER_ACTION(this->get_logger(), "Returning to travel position...");
    set_bucket_position(travel_position);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    LOGGER_SUCCESS(this->get_logger(), "Travel position reached");
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
      feedback->feedback_message = "Moving to home position";
      goal_handle->publish_feedback(feedback);
      home_bucket();

      result->success = true;
      result->message = "Homing completed successfully";
      goal_handle->succeed(result);
      LOGGER_SUCCESS(this->get_logger(), "Homing completed successfully");
    } catch (const std::exception & e)
    {
      result->success = false;
      result->message = std::string("Homing failed: ") + e.what();
      goal_handle->abort(result);
      LOGGER_FAILURE(this->get_logger(), "Homing failed: %s", e.what());
    }

    goal_active_ = false;
  }

  rclcpp_action::Server<Homing>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr bucket_position_pub_;
  bool goal_active_ = false;
};

/**
 * @brief Main function.
 * Initializes and runs the HomingServerSim node.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HomingServerSim>());
  rclcpp::shutdown();
  return 0;
}
