/**
 * @file localization_server.cpp
 * @author Grayson Arendt
 * @date 02/22/2026
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "lunabot_msgs/action/localization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// ANSI color codes
#define RESET "\033[0m"
#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define CYAN "\033[1;36m"

/**
 * @class LocalizationServer
 * @brief Handles localization by aligning the robot with tag 7 using transform lookups and publishes velocity commands.
 */
class LocalizationServer : public rclcpp::Node
{
public:
  using Localization = lunabot_msgs::action::Localization;
  using GoalHandleLocalization = rclcpp_action::ServerGoalHandle<Localization>;

  enum class State
  {
    SEARCHING,
    ALIGNING,
    COMPLETE
  };

  /**
   * @brief Constructor for LocalizationServer.
   */
  LocalizationServer()
    : Node("localization_server")
    , tf_buffer_(get_clock())
    , tf_listener_(tf_buffer_)
    , success_(false)
    , current_state_(State::SEARCHING)
  {
    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    action_server_ = rclcpp_action::create_server<Localization>(
        this, "localization_action",
        [this](const auto&, const auto&) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [this](const auto&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const auto goal_handle) { std::thread{ [this, goal_handle]() { execute(goal_handle); } }.detach(); });

    localization_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { localize(); });
    start_time_ = now();
  }

private:
  /**
   * @brief Executes the localization process.
   * @param goal_handle The handle to the goal being executed.
   */
  void execute(const std::shared_ptr<GoalHandleLocalization> goal_handle)
  {
    auto result = std::make_shared<Localization::Result>();

    while (!success_ && rclcpp::ok())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (success_)
    {
      result->x = depth_distance_ + 0.1;    // Center of robot is offset slightly
      result->y = lateral_distance_ + 1.0;  // Tag is located 1m away from corner
      result->success = true;
      goal_handle->succeed(result);
      rclcpp::shutdown();
    }
  }

  /**
   * @brief Core localization logic that aligns the robot with tag 7.
   * Uses two-phase approach: fast search, then precise alignment.
   */
  void localize()
  {
    // Timeout check
    if ((now() - start_time_).seconds() >= 30.0)
    {
      RCLCPP_ERROR(get_logger(), RED "LOCALIZATION TIMED OUT." RESET);
      cmd_vel_publisher_->publish(geometry_msgs::msg::Twist{});
      current_state_ = State::COMPLETE;
      return;
    }

    geometry_msgs::msg::Twist twist;

    try
    {
      auto tag7_to_d456 = tf_buffer_.lookupTransform("d456_link", "tag36h11:7", tf2::TimePointZero);
      auto d456_to_base = tf_buffer_.lookupTransform("base_link", "d456_link", tf2::TimePointZero);

      tf2::Transform tag7_to_d456_tf, d456_to_base_tf;
      tf2::fromMsg(tag7_to_d456.transform, tag7_to_d456_tf);
      tf2::fromMsg(d456_to_base.transform, d456_to_base_tf);

      tf2::Transform tag7_to_base_tf = d456_to_base_tf * tag7_to_d456_tf;

      depth_distance_ = -tag7_to_base_tf.getOrigin().x();
      lateral_distance_ = -tag7_to_base_tf.getOrigin().y();
      double yaw = tf2::getYaw(tag7_to_base_tf.getRotation());
      double yaw_error = std::abs(yaw - 1.57);

      // Tag is visible - switch to alignment mode
      if (current_state_ == State::SEARCHING)
      {
        RCLCPP_INFO(get_logger(), CYAN "TAG 7 DETECTED. SWITCHING TO ALIGNMENT MODE." RESET);
        current_state_ = State::ALIGNING;
      }

      // Check if aligned (within 0.05 rad = ~3 degrees)
      if (yaw_error < 0.05)
      {
        RCLCPP_INFO(get_logger(), GREEN "TAG 7 ALIGNED. LOCALIZATION SUCCESS!" RESET);
        success_ = true;
        current_state_ = State::COMPLETE;
        cmd_vel_publisher_->publish(geometry_msgs::msg::Twist{});
        return;
      }

      // Proportional control for smooth alignment (P-controller)
      double angular_speed = std::clamp(yaw_error * 1.5, 0.1, 0.5);
      twist.angular.z = (yaw < 1.57) ? angular_speed : -angular_speed;

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Aligning... Yaw error: %.3f rad, Speed: %.2f", yaw_error,
                           twist.angular.z);
    }
    catch (tf2::TransformException& ex)
    {
      // Tag not visible
      if (current_state_ == State::ALIGNING)
      {
        RCLCPP_WARN(get_logger(), YELLOW "LOST TAG 7. RETURNING TO SEARCH MODE." RESET);
        current_state_ = State::SEARCHING;
      }

      // Fast search rotation
      twist.angular.z = 0.5;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Searching for tag 7...");
    }

    cmd_vel_publisher_->publish(twist);
  }

  rclcpp_action::Server<Localization>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr localization_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Time start_time_;
  State current_state_;
  bool success_;
  double lateral_distance_, depth_distance_;
};

/**
 * @brief Main function.
 * Initializes and runs the LocalizationServer node.
 */
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationServer>());
  rclcpp::shutdown();
  return 0;
}
