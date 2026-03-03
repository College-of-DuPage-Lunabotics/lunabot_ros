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
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rmw/qos_profiles.h"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "lunabot_msgs/action/excavation.hpp"
#include "SparkMax.hpp"

// ANSI color codes
#define RESET "\033[0m"
#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define CYAN "\033[1;36m"

#define EXCAVATION_TICKS 1000.0
#define LIFT_TICKS 1000.0

/**
 * @class ExcavationServer
 * @brief Hardware excavation server that controls bucket actuators and manages excavation workflow.
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
    , left_actuator_motor_("can0", 2)
    , right_actuator_motor_("can0", 1)
    , vibration_motor_("can0", 5)
  {
    action_server_ = rclcpp_action::create_server<Excavation>(
        this, "excavation_action",
        [this](const auto&, const auto&) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
        [this](const auto&) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](const auto goal_handle) { std::thread{ [this, goal_handle]() { execute(goal_handle); } }.detach(); });

    auto selector_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    planner_publisher_ = this->create_publisher<std_msgs::msg::String>("/planner_selector", selector_qos);
    controller_publisher_ = this->create_publisher<std_msgs::msg::String>("/controller_selector", selector_qos);

    left_actuator_motor_.SetSensorType(SensorType::kEncoder);
    right_actuator_motor_.SetSensorType(SensorType::kEncoder);
    left_actuator_motor_.BurnFlash();
    right_actuator_motor_.BurnFlash();

    RCLCPP_INFO(this->get_logger(), GREEN "EXCAVATION SERVER INITIALIZED" RESET);
  }

private:
  /**
   * @brief Gets the average position of both bucket actuators.
   * @return Average encoder position in ticks.
   */
  double get_actuator_position()
  {
    left_actuator_motor_.Heartbeat();
    right_actuator_motor_.Heartbeat();
    double left_pos = left_actuator_motor_.GetPosition();
    double right_pos = right_actuator_motor_.GetPosition();
    // Subtract home offset to get position relative to home
    return (left_pos + right_pos) / 2.0 - home_offset_;
  }

  /**
   * @brief Updates the home offset from homing server parameter.
   */
  void update_home_offset()
  {
    auto homing_node = std::make_shared<rclcpp::Node>("homing_offset_reader");
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(homing_node, "/homing_server");
    if (param_client->wait_for_service(std::chrono::seconds(1)))
    {
      auto params = param_client->get_parameters({"actuator_home_offset"});
      if (!params.empty())
      {
        home_offset_ = params[0].as_double();
        RCLCPP_INFO(this->get_logger(), CYAN "Home offset updated: %.2f ticks" RESET, home_offset_);
      }
    }
  }

  /**
   * @brief Lowers bucket for excavation.
   */
  void lower_bucket()
  {
    RCLCPP_INFO(this->get_logger(), CYAN "LOWERING BUCKET..." RESET);

    double initial_position = get_actuator_position();
    double target_position = initial_position + EXCAVATION_TICKS;

    // Turn on vibration motor
    vibration_motor_.Heartbeat();
    vibration_motor_.SetDutyCycle(1.0);

    while (get_actuator_position() < target_position)
    {
      left_actuator_motor_.Heartbeat();
      right_actuator_motor_.Heartbeat();
      vibration_motor_.Heartbeat();
      left_actuator_motor_.SetDutyCycle(1.0);
      right_actuator_motor_.SetDutyCycle(1.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    left_actuator_motor_.Heartbeat();
    right_actuator_motor_.Heartbeat();
    left_actuator_motor_.SetDutyCycle(0.0);
    right_actuator_motor_.SetDutyCycle(0.0);

    // Turn off vibration motor
    vibration_motor_.Heartbeat();
    vibration_motor_.SetDutyCycle(0.0);

    double final_position = get_actuator_position();
    RCLCPP_INFO(this->get_logger(), GREEN "BUCKET LOWERED. Position: %.2f -> %.2f (delta: %.2f ticks)" RESET,
                initial_position, final_position, final_position - initial_position);
  }

  /**
   * @brief Lifts bucket up after excavation.
   */
  void lift_bucket()
  {
    RCLCPP_INFO(this->get_logger(), CYAN "LIFTING BUCKET..." RESET);

    double initial_position = get_actuator_position();
    double target_position = initial_position - LIFT_TICKS;

    // Turn on vibration motor
    vibration_motor_.Heartbeat();
    vibration_motor_.SetDutyCycle(1.0);

    while (get_actuator_position() > target_position)
    {
      left_actuator_motor_.Heartbeat();
      right_actuator_motor_.Heartbeat();
      vibration_motor_.Heartbeat();
      left_actuator_motor_.SetDutyCycle(-1.0);
      right_actuator_motor_.SetDutyCycle(-1.0);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    left_actuator_motor_.Heartbeat();
    right_actuator_motor_.Heartbeat();
    left_actuator_motor_.SetDutyCycle(0.0);
    right_actuator_motor_.SetDutyCycle(0.0);

    // Turn off vibration motor
    // Turn off vibration motor
    vibration_motor_.Heartbeat();
    vibration_motor_.SetDutyCycle(0.0);

    double final_position = get_actuator_position();
    RCLCPP_INFO(this->get_logger(), GREEN "BUCKET LIFTED. Position: %.2f -> %.2f (delta: %.2f ticks)" RESET,
                initial_position, final_position, final_position - initial_position);
  }

  /**
   * @brief Executes the excavation action sequence.
   * @param goal_handle Handle for the action goal.
   */
  void execute(const std::shared_ptr<GoalHandleExcavation> goal_handle)
  {
    auto result = std::make_shared<Excavation::Result>();
    bool excavation_success = false;

    // Update home offset before starting
    update_home_offset();

    lift_bucket();
    lower_bucket();

    try
    {
      // Switch to GridBased planner for obstacle traversal
      auto planner_msg = std_msgs::msg::String();
      planner_msg.data = "GridBased";
      planner_publisher_->publish(planner_msg);

      auto controller_msg = std_msgs::msg::String();
      controller_msg.data = "PurePursuit";
      controller_publisher_->publish(controller_msg);

      auto param_node = std::make_shared<rclcpp::Node>("param_helper");
      auto controller_params_client = std::make_shared<rclcpp::AsyncParametersClient>(param_node, "/controller_server");

      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(param_node);

      auto results = controller_params_client->set_parameters_atomically(
          { rclcpp::Parameter("goal_checker.xy_goal_tolerance", 0.2),
            rclcpp::Parameter("goal_checker.yaw_goal_tolerance", 0.2) });

      if (executor.spin_until_future_complete(results) != rclcpp::FutureReturnCode::SUCCESS)
      {
        throw std::runtime_error(RED "FAILED TO SET TOLERANCE PARAMETERS" RESET);
      }

      RCLCPP_INFO(this->get_logger(), CYAN "SENDING NAVIGATION GOAL TO CONSTRUCTION ZONE..." RESET);

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
        throw std::runtime_error(RED "FAILED TO SEND CONSTRUCTION GOAL" RESET);
      }
      auto goal_handle = excavation_goal.get();
      if (!goal_handle)
      {
        throw std::runtime_error(RED "CONSTRUCTION GOAL REJECTED BY SERVER" RESET);
      }

      if (nav_future.wait_for(std::chrono::seconds(360)) != std::future_status::ready)
      {
        throw std::runtime_error(RED "EXCAVATION TIMED OUT" RESET);
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), RED "EXCAVATION FAILED: %s" RESET, e.what());
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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_publisher_;

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
