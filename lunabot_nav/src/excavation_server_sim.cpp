/**
 * @file excavation_server_sim.cpp
 * @author Grayson Arendt
 * @date 02/22/2026
 */

#include "lunabot_logger/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "lunabot_msgs/action/excavation.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <chrono>
#include <memory>
#include <thread>

static constexpr double travel_position = -0.4;
static constexpr double excavate_position = 0.0;

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
  ExcavationServerSim() : Node("excavation_server")
  {
    action_server_ = rclcpp_action::create_server<Excavation>(
      this, "excavation_action",
      [this](const auto &, const auto &) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const auto &) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const auto goal_handle) {
        std::thread{[this, goal_handle]() { execute(goal_handle); }}.detach();
      });

    bucket_position_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>("/position_controller/commands", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    LOGGER_SUCCESS(this->get_logger(), "Excavation server (sim) initialized");
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
   * @brief Lowers bucket to excavation position.
   */
  void lower_bucket()
  {
    LOGGER_ACTION(this->get_logger(), "Lowering bucket to excavate (vibration on)...");
    set_bucket_position(excavate_position);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    LOGGER_SUCCESS(this->get_logger(), "Bucket lowered");
  }

  /**
   * @brief Drives robot forward to excavate material.
   */
  void drive_forward()
  {
    LOGGER_ACTION(this->get_logger(), "Driving forward to excavate...");

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

    LOGGER_SUCCESS(this->get_logger(), "Drive forward complete");
  }

  /**
   * @brief Lifts bucket to travel position.
   */
  void lift_bucket()
  {
    LOGGER_ACTION(this->get_logger(), "Lifting bucket (vibration off)...");
    set_bucket_position(travel_position);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    LOGGER_SUCCESS(this->get_logger(), "Bucket lifted");
  }

  /**
   * @brief Executes the excavation action sequence.
   * @param goal_handle Handle for the action goal.
   */
  void execute(const std::shared_ptr<GoalHandleExcavation> goal_handle)
  {
    if (goal_active_)
    {
      LOGGER_WARN(this->get_logger(), "Excavation already in progress");
      auto result = std::make_shared<Excavation::Result>();
      result->success = false;
      goal_handle->abort(result);
      return;
    }

    goal_active_ = true;
    LOGGER_SUCCESS(this->get_logger(), "Starting excavation sequence");

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
      LOGGER_SUCCESS(this->get_logger(), "Excavation completed successfully");
    } catch (const std::exception & e)
    {
      LOGGER_FAILURE(this->get_logger(), "Excavation failed: %s", e.what());
      result->success = false;
      goal_handle->abort(result);
    }

    goal_active_ = false;
  }

  rclcpp_action::Server<Excavation>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr bucket_position_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  bool goal_active_ = false;
};

/**
 * @brief Main function.
 * Initializes and runs the ExcavationServerSim node.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExcavationServerSim>());
  rclcpp::shutdown();
  return 0;
}
