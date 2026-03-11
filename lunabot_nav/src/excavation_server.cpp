/**
 * @file excavation_server.cpp
 * @author Grayson Arendt
 * @date 02/22/2026
 */

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "lunabot_msgs/action/excavation.hpp"
#include "lunabot_logger/logger.hpp"

#include "SparkMax.hpp"
#include <std_msgs/msg/float64.hpp>

#define EXCAVATION_POS 0.7
#define TRAVEL_POS 2.0

/**
 * @class ExcavationServer
 * @brief Hardware excavation server that controls bucket actuators and manages excavation sequence.
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
    : Node("excavation_server")
    , goal_active_(false)
    , home_offset_(0.0)
    , right_actuator_motor_("can0", 1)
    , left_actuator_motor_("can0", 2)
    , vibration_motor_("can0", 5)
  {
    action_server_ = rclcpp_action::create_server<Excavation>(
        this, "excavation_action",
        [this](const auto&, const auto&) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [this](const auto&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const auto goal_handle) { std::thread{ [this, goal_handle]() { execute(goal_handle); } }.detach(); });

    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    home_offset_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
        "actuator_home_offset", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
          home_offset_ = msg->data;
          LOGGER_ACTION(this->get_logger(), "Home offset updated: %.2f", home_offset_);
        });

    LOGGER_SUCCESS(this->get_logger(), "Excavation server initialized");
  }

private:
  /**
   * @brief Gets the position of the left bucket actuator (right encoder not working).
   * @return Left encoder position in radians.
   */
  double get_actuator_position()
  {
    left_actuator_motor_.Heartbeat();
    right_actuator_motor_.Heartbeat();
    double left_pos = left_actuator_motor_.GetPosition();
    // Subtract home offset to get position relative to home
    return left_pos - home_offset_;
  }

  /**
   * @brief Lowers bucket for excavation.
   */
  void lower_bucket()
  {
    LOGGER_ACTION(this->get_logger(), "Lowering bucket...");

    // Absolute target from zero: excavation position is below zero
    double target_position = -EXCAVATION_POS;

    // Turn on vibration motor
    vibration_motor_.Heartbeat();
    vibration_motor_.SetDutyCycle(1.0);

    while (get_actuator_position() > target_position)
    {
      left_actuator_motor_.Heartbeat();
      left_actuator_motor_.SetDutyCycle(1.0);
      right_actuator_motor_.SetDutyCycle(1.0);
    }

    left_actuator_motor_.Heartbeat();
    left_actuator_motor_.SetDutyCycle(0.0);
    right_actuator_motor_.SetDutyCycle(0.0);

    // Turn off vibration motor
    vibration_motor_.Heartbeat();
    vibration_motor_.SetDutyCycle(0.0);

    LOGGER_SUCCESS(this->get_logger(), "Bucket lowered to excavation position: %.2f", get_actuator_position());
  }

  /**
   * @brief Drives robot forward to excavate material.
   */
  void drive_forward()
  {
    LOGGER_ACTION(this->get_logger(), "Driving forward to excavate...");

    // Create velocity publisher for driving forward
    auto velocity_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Turn on vibration motor during collection
    vibration_motor_.Heartbeat();
    vibration_motor_.SetDutyCycle(1.0);

    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = 0.2; // Drive forward at 0.2 m/s

    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(3))
    {
      velocity_publisher->publish(twist_msg);
      vibration_motor_.Heartbeat();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop driving
    twist_msg.linear.x = 0.0;
    velocity_publisher->publish(twist_msg);

    // Turn off vibration motor
    vibration_motor_.SetDutyCycle(0.0);

    LOGGER_SUCCESS(this->get_logger(), "Drive forward complete");
  }

  /**
   * @brief Lifts bucket up after excavation.
   */
  void lift_bucket()
  {
    LOGGER_ACTION(this->get_logger(), "Lifting bucket...");

    // Absolute target from zero: travel position is above zero
    double target_position = TRAVEL_POS;

    // Turn on vibration motor
    vibration_motor_.Heartbeat();
    vibration_motor_.SetDutyCycle(1.0);

    while (get_actuator_position() < target_position)
    {
      left_actuator_motor_.Heartbeat();
      right_actuator_motor_.Heartbeat();
      vibration_motor_.Heartbeat();
      left_actuator_motor_.SetDutyCycle(-1.0);
      right_actuator_motor_.SetDutyCycle(-1.0);
    }

    left_actuator_motor_.Heartbeat();
    right_actuator_motor_.Heartbeat();
    left_actuator_motor_.SetDutyCycle(0.0);
    right_actuator_motor_.SetDutyCycle(0.0);

    // Turn off vibration motor
    vibration_motor_.Heartbeat();
    vibration_motor_.SetDutyCycle(0.0);

    LOGGER_SUCCESS(this->get_logger(), "Bucket lifted to travel position: %.2f", get_actuator_position());
  }

  /**
   * @brief Executes the excavation action sequence.
   * @param goal_handle Handle for the action goal.
   */
  void execute(const std::shared_ptr<GoalHandleExcavation> goal_handle)
  {
    auto result = std::make_shared<Excavation::Result>();
    bool excavation_success = false;

    lift_bucket();
    lower_bucket();
    drive_forward();
    lift_bucket();

    try
    {
      auto param_node = std::make_shared<rclcpp::Node>("param_helper");
      auto controller_params_client = std::make_shared<rclcpp::AsyncParametersClient>(param_node, "/controller_server");

      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(param_node);

      auto results = controller_params_client->set_parameters_atomically(
          { rclcpp::Parameter("goal_checker.xy_goal_tolerance", 0.2),
            rclcpp::Parameter("goal_checker.yaw_goal_tolerance", 0.2) });

      if (executor.spin_until_future_complete(results) != rclcpp::FutureReturnCode::SUCCESS)
      {
        throw std::runtime_error(RED "Failed to set tolerance parameters" RESET);
      }

      LOGGER_ACTION(this->get_logger(), "Sending navigation goal to construction zone...");

      auto goal_msg = NavigateToPose::Goal();
      geometry_msgs::msg::Pose goal_pose;

      // Target coordinates for construction zone
      // 4.3, -0.2 for KSC
      // 6.1, 0.5 for UCF
      goal_pose.position.x = 4.3;
      goal_pose.position.y = -0.2;
      goal_pose.orientation.z = 0.707;
      goal_pose.orientation.w = 0.707;

      goal_msg.pose.pose = goal_pose;
      goal_msg.pose.header.stamp = this->now();
      goal_msg.pose.header.frame_id = "map";

      auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
      std::promise<bool> nav_completed;
      std::future<bool> nav_future = nav_completed.get_future();

      send_goal_options.result_callback = [&](const GoalHandleNavigate::WrappedResult& result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
          excavation_success = true;
          lift_bucket();
        }
        else
        {
          excavation_success = false;
          lift_bucket();
        }
        nav_completed.set_value(true);
      };

      auto excavation_goal = navigation_client_->async_send_goal(goal_msg, send_goal_options);

      if (excavation_goal.wait_for(std::chrono::seconds(2)) != std::future_status::ready)
      {
        throw std::runtime_error(RED "Failed to send construction goal" RESET);
      }
      auto goal_handle = excavation_goal.get();
      if (!goal_handle)
      {
        throw std::runtime_error(RED "Construction goal rejected by server" RESET);
      }

      if (nav_future.wait_for(std::chrono::seconds(360)) != std::future_status::ready)
      {
        throw std::runtime_error(RED "Excavation timed out" RESET);
      }
    }
    catch (const std::exception& e)
    {
      LOGGER_FAILURE(this->get_logger(), "Excavation failed: %s", e.what());
      excavation_success = false;
    }

    result->success = excavation_success;
    if (excavation_success)
    {
      goal_handle->succeed(result);
    }
    else
    {
      goal_handle->abort(result);
    }
  }

  rclcpp_action::Server<Excavation>::SharedPtr action_server_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr home_offset_subscriber_;

  double home_offset_;
  SparkMax left_actuator_motor_;
  SparkMax right_actuator_motor_;
  SparkMax vibration_motor_;
  bool goal_active_;
};

/**
 * @brief Main function.
 * Initializes and runs the ExcavationServer node.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExcavationServer>());
  rclcpp::shutdown();
  return 0;
}
