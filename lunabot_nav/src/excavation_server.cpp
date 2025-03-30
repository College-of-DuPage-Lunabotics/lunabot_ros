/**
 * @file excavation_server.cpp
 * @author Grayson Arendt
 * @date 02/15/2025
 */

#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rmw/qos_profiles.h"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "lunabot_msgs/action/excavation.hpp"

/**
 * @class ExcavationServer
 * @brief Handles excavation requests and manages the excavation process.
 */
class ExcavationServer : public rclcpp::Node
{
public:
  using Excavation = lunabot_msgs::action::Excavation;
  using GoalHandleExcavation = rclcpp_action::ServerGoalHandle<Excavation>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  /**
   * @brief Constructor for the ExcavationServer class.
   */
  ExcavationServer()
  : Node("excavation_server"), goal_active_(false)
  {
    action_server_ = rclcpp_action::create_server<Excavation>(
      this, "excavation_action",
      [this](const auto &, const auto &) {return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;},
      [this](const auto &) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](const auto goal_handle) {
        std::thread{[this, goal_handle]() {
            execute(goal_handle);
          }}.detach();
      });

    auto selector_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability(
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    planner_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "/planner_selector",
      selector_qos);
    controller_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "/controller_selector",
      selector_qos);
  }

private:
  /**
   * @brief Executes the excavation process.
   * @param goal_handle The handle to the goal being executed.
   */
  void execute(const std::shared_ptr<GoalHandleExcavation> goal_handle)
  {
    auto result = std::make_shared<Excavation::Result>();
    bool excavation_success = false;

    try {
      auto planner_msg = std_msgs::msg::String();
      planner_msg.data = "StraightLine";
      planner_publisher_->publish(planner_msg);

      auto controller_msg = std_msgs::msg::String();
      controller_msg.data = "PurePursuit";
      controller_publisher_->publish(controller_msg);

      auto param_node = std::make_shared<rclcpp::Node>("param_helper");
      auto controller_params_client =
        std::make_shared<rclcpp::AsyncParametersClient>(param_node, "/controller_server");

      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(param_node);

      auto results = controller_params_client->set_parameters_atomically(
        {rclcpp::Parameter("goal_checker.xy_goal_tolerance", 0.2),
          rclcpp::Parameter("goal_checker.yaw_goal_tolerance", 0.2)});

      if (executor.spin_until_future_complete(results) != rclcpp::FutureReturnCode::SUCCESS) {
        throw std::runtime_error("\033[1;31mFAILED TO SET TOLERANCE PARAMETERS\033[0m");
      }

      RCLCPP_INFO(
        this->get_logger(), "\033[1;36mSENDING NAVIGATION GOAL TO CONSTRUCTION ZONE...\033[0m");

      auto goal_msg = NavigateToPose::Goal();
      geometry_msgs::msg::Pose goal_pose;

      goal_pose.position.x = 4.3;
      goal_pose.position.y = -0.4;
      goal_pose.orientation.z = 0.707;
      goal_pose.orientation.w = 0.707;

      goal_msg.pose.pose = goal_pose;
      goal_msg.pose.header.stamp = this->now();
      goal_msg.pose.header.frame_id = "map";

      auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
      std::promise<bool> nav_completed;
      std::future<bool> nav_future = nav_completed.get_future();

      send_goal_options.result_callback = [&](const GoalHandleNavigate::WrappedResult & result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            excavation_success = true;
          } else {
            excavation_success = false;
          }
          nav_completed.set_value(true);
        };

      auto excavation_goal = navigation_client_->async_send_goal(goal_msg, send_goal_options);

      if (excavation_goal.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
        throw std::runtime_error("\033[1;31mFAILED TO SEND CONSTRUCTION GOAL\033[0m");
      }
      auto goal_handle = excavation_goal.get();
      if (!goal_handle) {
        throw std::runtime_error("\033[1;31mCONSTRUCTION GOAL REJECTED BY SERVER\033[0m");
      }

      if (nav_future.wait_for(std::chrono::seconds(360)) != std::future_status::ready) {
        throw std::runtime_error("\033[1;31mEXCAVATION TIMED OUT\033[0m");
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "\033[1;31mEXCAVATION FAILED: %s\033[0m", e.what());
      excavation_success = false;
    }

    result->success = excavation_success;
    if (excavation_success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  rclcpp_action::Server<Excavation>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_publisher_;

  bool goal_active_;
};

/**
 * @brief Main function.
 * Initializes and runs the ExcavationServer node.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExcavationServer>());
  rclcpp::shutdown();
  return 0;
}
