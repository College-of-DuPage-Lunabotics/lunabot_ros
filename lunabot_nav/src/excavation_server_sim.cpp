/**
 * @file excavation_server_sim.cpp
 * @author Grayson Arendt
 * @date 02/22/2026
 */

#include <chrono>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "lunabot_msgs/action/excavation.hpp"

// ANSI color codes
#define RESET "\033[0m"
#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define CYAN "\033[1;36m"

#define TRAVEL_POSITION -0.4
#define EXCAVATE_POSITION 0.0

/**
 * @class ExcavationServerSim
 * @brief Simulation-compatible excavation server that controls bucket joint position.
 */
class ExcavationServerSim : public rclcpp::Node
{
public:
  using Excavation = lunabot_msgs::action::Excavation;
  using GoalHandleExcavation = rclcpp_action::ServerGoalHandle<Excavation>;

  /**
   * @brief Constructor for the ExcavationServerSim class.
   */
  ExcavationServerSim()
    : Node("excavation_server")
    , goal_active_(false)
  {
    action_server_ = rclcpp_action::create_server<Excavation>(
        this, "excavation_action",
        [this](const auto&, const auto&) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [this](const auto&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const auto goal_handle) { std::thread{ [this, goal_handle]() { execute(goal_handle); } }.detach(); });

    bucket_position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/position_controller/commands", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), GREEN "EXCAVATION SERVER (SIM) INITIALIZED" RESET);
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
   * @brief Lowers bucket to excavation position.
   */
  void lower_bucket()
  {
    RCLCPP_INFO(this->get_logger(), CYAN "LOWERING BUCKET TO EXCAVATE (vibration ON)..." RESET);
    set_bucket_position(EXCAVATE_POSITION);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(this->get_logger(), GREEN "BUCKET LOWERED" RESET);
  }

  /**
   * @brief Drives robot forward to dig into material.
   */
  void drive_forward()
  {
    RCLCPP_INFO(this->get_logger(), CYAN "DRIVING FORWARD TO DIG..." RESET);

    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0.2;

    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(3))
    {
      cmd_vel_pub_->publish(twist_msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    twist_msg.linear.x = 0.0;
    cmd_vel_pub_->publish(twist_msg);

    RCLCPP_INFO(this->get_logger(), GREEN "DRIVE FORWARD COMPLETE" RESET);
  }

  /**
   * @brief Lifts bucket to travel position.
   */
  void lift_bucket()
  {
    RCLCPP_INFO(this->get_logger(), CYAN "LIFTING BUCKET (vibration OFF)..." RESET);
    set_bucket_position(TRAVEL_POSITION);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    RCLCPP_INFO(this->get_logger(), GREEN "BUCKET LIFTED" RESET);
  }

  /**
   * @brief Executes the excavation action sequence.
   * @param goal_handle Handle for the action goal.
   */
  void execute(const std::shared_ptr<GoalHandleExcavation> goal_handle)
  {
    if (goal_active_)
    {
      RCLCPP_WARN(this->get_logger(), YELLOW "EXCAVATION ALREADY IN PROGRESS" RESET);
      auto result = std::make_shared<Excavation::Result>();
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    goal_active_ = true;
    RCLCPP_INFO(this->get_logger(), GREEN "STARTING EXCAVATION SEQUENCE" RESET);

    auto result = std::make_shared<Excavation::Result>();

    try
    {
      // Lower bucket to ground
      lower_bucket();
      // Drive forward to collect material
      drive_forward();
      // Lift bucket back to travel position
      lift_bucket();

      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), GREEN "EXCAVATION COMPLETED SUCCESSFULLY" RESET);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), RED "EXCAVATION FAILED: %s" RESET, e.what());
      result->success = false;
      goal_handle->abort(result);
    }

    goal_active_ = false;
  }

  rclcpp_action::Server<Excavation>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr bucket_position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  bool goal_active_;
};

/**
 * @brief Main function.
 * Initializes and runs the ExcavationServerSim node.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExcavationServerSim>());
  rclcpp::shutdown();
  return 0;
}
