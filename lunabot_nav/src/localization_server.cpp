/**
 * @file localization_server.cpp
 * @author Grayson Arendt
 * @date 03/29/2025
 */

#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "lunabot_msgs/action/localization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class LocalizationServer : public rclcpp::Node
{
public:
  using Localization = lunabot_msgs::action::Localization;
  using GoalHandleLocalization = rclcpp_action::ServerGoalHandle<Localization>;

  LocalizationServer()
  : Node("localization_server"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_), success_(false),
    turn_direction_set_(false)
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
  void execute(const std::shared_ptr<GoalHandleLocalization> goal_handle)
  {
    auto result = std::make_shared<Localization::Result>();

    while (!success_ && rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (success_) {
      result->x = depth_distance_ + 0.1;
      result->y = lateral_distance_ + 1.0; // Tag is 1m away from corner
      result->success = true;
      goal_handle->succeed(result);
      rclcpp::shutdown();
    }
  }

  void localize()
  {
    geometry_msgs::msg::Twist twist;

    try {
      auto oak_d_to_base =
        tf_buffer_.lookupTransform("base_link", "oak_d_link", tf2::TimePointZero);

      geometry_msgs::msg::TransformStamped tag7_to_oak_d;
      try {
        tag7_to_oak_d = tf_buffer_.lookupTransform("oak_d_link", "tag36h11:7", tf2::TimePointZero);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TAG 7 NOT VISIBLE BY OAK-D: %s", ex.what());
        return;
      }

      if (!turn_direction_set_) {
        try {
          tf_buffer_.lookupTransform("d456_link", "tag36h11:11", tf2::TimePointZero);
          turn_clockwise_ = false;
        } catch (tf2::TransformException &) {
          turn_clockwise_ = true;
        }

        turn_direction_set_ = true;
      }

      tf2::Transform tag7_to_oak_d_tf;
      tf2::fromMsg(tag7_to_oak_d.transform, tag7_to_oak_d_tf);

      tf2::Transform oak_d_to_base_tf;
      tf2::fromMsg(oak_d_to_base.transform, oak_d_to_base_tf);

      tf2::Transform tag7_to_base_tf = oak_d_to_base_tf * tag7_to_oak_d_tf;

      depth_distance_ = -tag7_to_base_tf.getOrigin().x();
      lateral_distance_ = -tag7_to_base_tf.getOrigin().y();

      double yaw = tf2::getYaw(tag7_to_base_tf.getRotation());

      if (std::abs(yaw - 1.57) < 0.05) {
        twist.angular.z = 0.0;
        success_ = true;
      } else {
        twist.angular.z = turn_clockwise_ ? 0.3 : -0.3;
      }

      if (!success_ && (now() - start_time_).seconds() < 30.0) {
        cmd_vel_publisher_->publish(twist);
      } else if (!success_) {
        RCLCPP_WARN_ONCE(get_logger(), "\033[1;33mLOCALIZATION TIMED OUT.\033[0m");
      } else {
        cmd_vel_publisher_->publish(geometry_msgs::msg::Twist{});
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
  bool success_, turn_direction_set_, turn_clockwise_;
  double lateral_distance_, depth_distance_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationServer>());
  rclcpp::shutdown();
  return 0;
}
