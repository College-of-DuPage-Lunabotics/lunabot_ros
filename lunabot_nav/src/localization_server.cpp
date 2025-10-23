/**
 * @file localization_server.cpp
 * @author Grayson Arendt
 * @date 03/29/2025
 */

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

/**
 * @class LocalizationServer
 * @brief Handles localization by aligning the robot with tag 7 using transform lookups and publishes velocity commands.
 */
class LocalizationServer : public rclcpp::Node
{
public:
  using Localization = lunabot_msgs::action::Localization;
  using GoalHandleLocalization = rclcpp_action::ServerGoalHandle<Localization>;

  /**
   * @brief Constructor for LocalizationServer.
   */
  LocalizationServer()
  : Node("localization_server"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_), success_(false)
  {
    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    action_server_ = rclcpp_action::create_server<Localization>(
      this, "localization_action",
      [this](const auto &, const auto &) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [this](const auto &) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const auto goal_handle) {
        std::thread{[this, goal_handle]() {
            execute(goal_handle);
          }}.detach();
      });

    localization_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {localize();});
    start_time_ = now();
  }

private:
  /**
   * @brief Executes the excavation process.
   * @param goal_handle The handle to the goal being executed.
   */
  void execute(const std::shared_ptr<GoalHandleLocalization> goal_handle)
  {
    auto result = std::make_shared<Localization::Result>();

    while (!success_ && rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (success_) {
      result->x = depth_distance_ + 0.1;         // Center of robot is offset slightly
      result->y = lateral_distance_ + 1.0;       // Tag is located 1m away from corner
      result->success = true;
      goal_handle->succeed(result);
      rclcpp::shutdown();
    }
  }

  /**
   * @brief Core localization logic that aligns the robot with tag 7.
   * Uses D456 camera to detect and align with tag 7.
   * The robot spins until it's properly aligned with tag 7 facing east.
   */
  void localize()
  {
    geometry_msgs::msg::Twist twist;

    try {
      // Check if D456 sees tag 7 and is properly aligned
      try {
        auto tag7_to_d456 =
          tf_buffer_.lookupTransform("d456_link", "tag36h11:7", tf2::TimePointZero);
        auto d456_to_base =
          tf_buffer_.lookupTransform("base_link", "d456_link", tf2::TimePointZero);

        tf2::Transform tag7_to_d456_tf;
        tf2::fromMsg(tag7_to_d456.transform, tag7_to_d456_tf);

        tf2::Transform d456_to_base_tf;
        tf2::fromMsg(d456_to_base.transform, d456_to_base_tf);

        tf2::Transform tag7_to_base_tf = d456_to_base_tf * tag7_to_d456_tf;

        depth_distance_ = -tag7_to_base_tf.getOrigin().x();
        lateral_distance_ = -tag7_to_base_tf.getOrigin().y();
        double yaw = tf2::getYaw(tag7_to_base_tf.getRotation());

        // Check if the yaw is within the desired range
        if (std::abs(yaw - 1.57) < 0.05) { // 1.57 radians = 90 degrees (east)
          RCLCPP_INFO(get_logger(), "TAG 7 VISIBLE AND ALIGNED: Localization successful.");
          success_ = true;
          cmd_vel_publisher_->publish(geometry_msgs::msg::Twist{});           // Stop the robot
          return;           // Exit early since localization is successful
        } else {
          RCLCPP_INFO(
            get_logger(), "TAG 7 VISIBLE BUT NOT ALIGNED: Current yaw = %.2f, target = 1.57", yaw);
          // Continue spinning to align properly
        }
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TAG 7 NOT VISIBLE: %s", ex.what());
      }

      // Rotate the robot to search for and align with tag 7
      twist.angular.z = 0.3;
      cmd_vel_publisher_->publish(twist);

      // Timeout logic
      if ((now() - start_time_).seconds() >= 30.0) {
        RCLCPP_WARN_ONCE(get_logger(), "\033[1;33mLOCALIZATION TIMED OUT.\033[0m");
        cmd_vel_publisher_->publish(geometry_msgs::msg::Twist{});         // Stop the robot
      }
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TRANSFORM UNAVAILABLE: %s", ex.what());
    }
  }

  rclcpp_action::Server<Localization>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr localization_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Time start_time_;
  bool success_;
  double lateral_distance_, depth_distance_;
};

/**
 * @brief Main function.
 * Initializes and runs the LocalizationServer node.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationServer>());
  rclcpp::shutdown();
  return 0;
}
