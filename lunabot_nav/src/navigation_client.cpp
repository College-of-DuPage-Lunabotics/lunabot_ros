/**
 * @file navigation_client.cpp
 * @author Grayson Arendt
 * @date 12/19/2024
 */

#include <chrono>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rmw/qos_profiles.h"
#include "std_msgs/msg/string.hpp"

#include "lunabot_msgs/action/excavation.hpp"
#include "lunabot_msgs/action/localization.hpp"

// ANSI color codes
#define RESET "\033[0m"
#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define CYAN "\033[1;36m"

/**
 * @class NavigationClient
 * @brief Handles localization responses and sends navigation and excavation requests.
 */
class NavigationClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Localization = lunabot_msgs::action::Localization;
  using GoalHandleLocalization = rclcpp_action::ClientGoalHandle<Localization>;
  using Excavation = lunabot_msgs::action::Excavation;
  using GoalHandleExcavation = rclcpp_action::ClientGoalHandle<Excavation>;

  enum class State
  {
    IDLE,
    LOCALIZATION,
    NAVIGATING_TO_EXCAVATION,
    EXCAVATING,
    COMPLETE
  };

  /**
   * @brief Constructor for the NavigationClient class.
   */
  NavigationClient() : Node("navigation_client")
  {
    // Declare parameter for whether to use localization
    this->declare_parameter<bool>("use_localization", false);
    bool use_localization = this->get_parameter("use_localization").as_bool();

    // Set initial state based on whether localization is needed
    current_state_ = use_localization ? State::LOCALIZATION : State::NAVIGATING_TO_EXCAVATION;

    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    localization_client_ = rclcpp_action::create_client<Localization>(this, "localization_action");
    excavation_client_ = rclcpp_action::create_client<Excavation>(this, "excavation_action");

    auto selector_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    planner_publisher_ = this->create_publisher<std_msgs::msg::String>("/planner_selector", selector_qos);
    controller_publisher_ = this->create_publisher<std_msgs::msg::String>("/controller_selector", selector_qos);

    execution_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&NavigationClient::execute, this));
  }

private:
  /**
   * @brief Runs the main execution sequence based on current state.
   */
  void execute()
  {
    switch (current_state_)
    {
      case State::IDLE:
        break;
      case State::LOCALIZATION:
        request_localization();
        current_state_ = State::IDLE;
        break;
      case State::NAVIGATING_TO_EXCAVATION:
        request_navigation();
        current_state_ = State::IDLE;
        break;
      case State::EXCAVATING:
        request_excavation();
        current_state_ = State::IDLE;
        break;
      case State::COMPLETE:
        RCLCPP_INFO(this->get_logger(), GREEN "CYCLE COMPLETE!" RESET);
        execution_timer_->cancel();
        break;
    }
  }

  /**
   * @brief Sends localization request to localization server.
   */
  void request_localization()
  {
    if (!localization_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
      RCLCPP_WARN_ONCE(this->get_logger(), YELLOW "LOCALIZATION ACTION SERVER NOT AVAILABLE." RESET);
      return;
    }

    auto goal_msg = Localization::Goal();
    auto send_goal_options = rclcpp_action::Client<Localization>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&NavigationClient::handle_localization_result, this, std::placeholders::_1);

    localization_client_->async_send_goal(goal_msg, send_goal_options);
  }

  /**
   * @brief Handles the result from the localization server.
   * @param result The result from the localization action.
   */
  void handle_localization_result(const GoalHandleLocalization::WrappedResult& result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      initial_x_ = result.result->x;
      initial_y_ = result.result->y;
      RCLCPP_INFO(this->get_logger(), "\033[1;32mLOCALIZATION COMPLETE. INITIAL POSE: [%.2f, %.2f]\033[0m", initial_x_,
                  initial_y_);
      current_state_ = State::NAVIGATING_TO_EXCAVATION;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "\033[1;31mLOCALIZATION FAILED.\033[0m");
      rclcpp::shutdown();
    }
  }

  /**
   * @brief Sends goal request to navigation server.
   */
  void request_navigation()
  {
    // Wait for Nav2 action server to be ready (up to 30 seconds)
    RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server to be ready...");
    if (!navigation_client_->wait_for_action_server(std::chrono::seconds(30)))
    {
      RCLCPP_ERROR(this->get_logger(), RED "NAVIGATION ACTION SERVER NOT AVAILABLE AFTER 30s." RESET);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Nav2 action server ready!");

    // Use StraightLine planner for clear path to excavation zone
    auto planner_msg = std_msgs::msg::String();
    planner_msg.data = "StraightLine";
    planner_publisher_->publish(planner_msg);

    auto controller_msg = std_msgs::msg::String();
    controller_msg.data = "PurePursuit";
    controller_publisher_->publish(controller_msg);

    auto goal_msg = NavigateToPose::Goal();
    geometry_msgs::msg::Pose goal_pose;

    // Short travel out of starting zone to excavation area
    goal_pose.position.x = 3.0;
    goal_pose.position.y = 1.6;
    goal_pose.orientation.z = 0.707;
    goal_pose.orientation.w = 0.707;

    goal_msg.pose.pose = goal_pose;
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = "map";

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&NavigationClient::handle_navigation_result, this, std::placeholders::_1);

    RCLCPP_INFO(this->get_logger(), "\033[1;36mSENDING NAVIGATION GOAL TO EXCAVATION ZONE...\033[0m");
    navigation_client_->async_send_goal(goal_msg, send_goal_options);
  }

  /**
   * @brief Callback for the result of the navigation goal.
   * @param result The result of the goal execution.
   */
  void handle_navigation_result(const GoalHandleNavigate::WrappedResult& result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(this->get_logger(), GREEN "EXCAVATION ZONE REACHED. REQUESTING EXCAVATION..." RESET);
      current_state_ = State::EXCAVATING;
    }
    else if (result.code == rclcpp_action::ResultCode::ABORTED)
    {
      RCLCPP_ERROR(this->get_logger(), RED "GOAL ABORTED, UNABLE TO REACH EXCAVATION ZONE" RESET);
      current_state_ = State::IDLE;
    }
  }

  /**
   * @brief Sends a request to the excavation server.
   */
  void request_excavation()
  {
    if (!excavation_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
      RCLCPP_WARN_ONCE(this->get_logger(), YELLOW "EXCAVATION ACTION SERVER NOT AVAILABLE." RESET);
      return;
    }

    auto goal_msg = Excavation::Goal();
    auto send_goal_options = rclcpp_action::Client<Excavation>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&NavigationClient::handle_excavation_result, this, std::placeholders::_1);

    excavation_client_->async_send_goal(goal_msg, send_goal_options);
  }

  /**
   * @brief Callback for the result of the excavation action.
   * @param result The result of the excavation action.
   */
  void handle_excavation_result(const GoalHandleExcavation::WrappedResult& result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(this->get_logger(), GREEN "EXCAVATION SUCCESS!" RESET);
      current_state_ = State::COMPLETE;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), RED "EXCAVATION FAILED." RESET);
      current_state_ = State::IDLE;
    }
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp_action::Client<Localization>::SharedPtr localization_client_;
  rclcpp_action::Client<Excavation>::SharedPtr excavation_client_;
  rclcpp::TimerBase::SharedPtr execution_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planner_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr controller_publisher_;

  State current_state_;
  double initial_x_, initial_y_;
};

/**
 * @brief Main function.
 * Initializes and runs the NavigationClient node.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationClient>());
  rclcpp::shutdown();
  return 0;
}
