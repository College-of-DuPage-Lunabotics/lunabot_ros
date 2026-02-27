/**
 * @file navigation_client.cpp
 * @author Grayson Arendt
 * @date 02/22/2026
 */

#include <chrono>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "lunabot_msgs/action/excavation.hpp"
#include "lunabot_msgs/action/depositing.hpp"
#include "lunabot_msgs/action/localization.hpp"

// ANSI color codes
#define RESET "\033[0m"
#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define CYAN "\033[1;36m"

/**
 * @class NavigationClient
 * @brief Runs autonomous one cycle sequence: localization, navigation, excavation, and depositing actions.
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
  using Depositing = lunabot_msgs::action::Depositing;
  using GoalHandleDepositing = rclcpp_action::ClientGoalHandle<Depositing>;

  enum class State
  {
    IDLE,
    LOCALIZATION,
    NAVIGATING_TO_EXCAVATION,
    EXCAVATING,
    NAVIGATING_TO_CONSTRUCTION,
    DEPOSITING,
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
    depositing_client_ = rclcpp_action::create_client<Depositing>(this, "depositing_action");

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
      case State::NAVIGATING_TO_CONSTRUCTION:
        request_navigation_to_construction();
        current_state_ = State::IDLE;
        break;
      case State::DEPOSITING:
        request_depositing();
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
      current_state_ = State::IDLE;
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

    auto goal_msg = NavigateToPose::Goal();
    geometry_msgs::msg::Pose goal_pose;

    // Short travel out of starting zone to excavation area
    goal_pose.position.x = 0.0;
    goal_pose.position.y = -2.0;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = -0.7071068;
    goal_pose.orientation.w = 0.7071068;

    goal_msg.pose.pose = goal_pose;
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = "map";

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&NavigationClient::handle_navigation_result, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NavigationClient::handle_navigation_feedback, this, std::placeholders::_1, std::placeholders::_2);

    RCLCPP_INFO(this->get_logger(), "\033[1;36mSENDING NAVIGATION GOAL TO EXCAVATION ZONE [0.0, -2.0]...\033[0m");
    navigation_client_->async_send_goal(goal_msg, send_goal_options);
  }

  /**
   * @brief Callback for navigation feedback (distance to goal updates).
   * @param goal_handle The goal handle.
   * @param feedback The feedback message containing distance remaining.
   */
  void handle_navigation_feedback(GoalHandleNavigate::SharedPtr,
                                   const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Distance remaining: %.2f meters", feedback->distance_remaining);
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
    else if (result.code == rclcpp_action::ResultCode::CANCELED)
    {
      RCLCPP_WARN(this->get_logger(), YELLOW "NAVIGATION TO EXCAVATION ZONE CANCELED" RESET);
      current_state_ = State::IDLE;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), RED "NAVIGATION TO EXCAVATION ZONE FAILED WITH UNKNOWN RESULT" RESET);
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
      RCLCPP_INFO(this->get_logger(), GREEN "EXCAVATION SUCCESS! NAVIGATING TO CONSTRUCTION ZONE..." RESET);
      current_state_ = State::NAVIGATING_TO_CONSTRUCTION;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), RED "EXCAVATION FAILED." RESET);
      current_state_ = State::IDLE;
    }
  }

  /**
   * @brief Sends goal request to navigation server for construction zone.
   */
  void request_navigation_to_construction()
  {
    // Wait for Nav2 action server to be ready
    RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 action server to be ready...");
    if (!navigation_client_->wait_for_action_server(std::chrono::seconds(30)))
    {
      RCLCPP_ERROR(this->get_logger(), RED "NAVIGATION ACTION SERVER NOT AVAILABLE AFTER 30s." RESET);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Nav2 action server ready!");

    auto goal_msg = NavigateToPose::Goal();
    geometry_msgs::msg::Pose goal_pose;

    // Navigate to construction zone coordinates (rotated 180 from excavation for depositing)
    goal_pose.position.x = -4.5;
    goal_pose.position.y = 0.4;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = -0.7071068;
    goal_pose.orientation.w = 0.7071068;

    goal_msg.pose.pose = goal_pose;
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = "map";

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&NavigationClient::handle_construction_navigation_result, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NavigationClient::handle_construction_feedback, this, std::placeholders::_1, std::placeholders::_2);

    RCLCPP_INFO(this->get_logger(), "\033[1;36mSENDING NAVIGATION GOAL TO CONSTRUCTION ZONE [-4.5, 0.4]...\033[0m");
    navigation_client_->async_send_goal(goal_msg, send_goal_options);
  }

  /**
   * @brief Callback for navigation feedback to construction zone.
   * @param goal_handle The goal handle.
   * @param feedback The feedback message containing distance remaining.
   */
  void handle_construction_feedback(GoalHandleNavigate::SharedPtr,
                                     const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Distance to construction zone: %.2f meters", feedback->distance_remaining);
  }

  /**
   * @brief Callback for the result of navigation to construction zone.
   * @param result The result of the goal execution.
   */
  void handle_construction_navigation_result(const GoalHandleNavigate::WrappedResult& result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(this->get_logger(), GREEN "CONSTRUCTION ZONE REACHED. REQUESTING DUMP..." RESET);
      current_state_ = State::DEPOSITING;
    }
    else if (result.code == rclcpp_action::ResultCode::ABORTED)
    {
      RCLCPP_ERROR(this->get_logger(), RED "GOAL ABORTED, UNABLE TO REACH CONSTRUCTION ZONE" RESET);
      current_state_ = State::IDLE;
    }
    else if (result.code == rclcpp_action::ResultCode::CANCELED)
    {
      RCLCPP_WARN(this->get_logger(), YELLOW "NAVIGATION TO CONSTRUCTION ZONE CANCELED" RESET);
      current_state_ = State::IDLE;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), RED "NAVIGATION TO CONSTRUCTION ZONE FAILED WITH UNKNOWN RESULT" RESET);
      current_state_ = State::IDLE;
    }
  }

  /**
   * @brief Sends a request to the depositing server.
   */
  void request_depositing()
  {
    if (!depositing_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
      RCLCPP_WARN_ONCE(this->get_logger(), YELLOW "DEPOSITING ACTION SERVER NOT AVAILABLE." RESET);
      return;
    }

    auto goal_msg = Depositing::Goal();
    auto send_goal_options = rclcpp_action::Client<Depositing>::SendGoalOptions();
    send_goal_options.result_callback =
        std::bind(&NavigationClient::handle_depositing_result, this, std::placeholders::_1);

    depositing_client_->async_send_goal(goal_msg, send_goal_options);
  }

  /**
   * @brief Callback for the result of the depositing action.
   * @param result The result of the depositing action.
   */
  void handle_depositing_result(const GoalHandleDepositing::WrappedResult& result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(this->get_logger(), GREEN "DEPOSITING SUCCESS! CYCLE COMPLETE!" RESET);
      current_state_ = State::COMPLETE;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), RED "DEPOSITING FAILED." RESET);
      current_state_ = State::IDLE;
    }
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp_action::Client<Localization>::SharedPtr localization_client_;
  rclcpp_action::Client<Excavation>::SharedPtr excavation_client_;
  rclcpp_action::Client<Depositing>::SharedPtr depositing_client_;
  rclcpp::TimerBase::SharedPtr execution_timer_;

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
