/**
 * @file navigation_client.cpp
 * @author Grayson Arendt
 * @date 02/22/2026
 */

#include "lunabot_logger/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "lunabot_msgs/action/depositing.hpp"
#include "lunabot_msgs/action/excavation.hpp"
#include "lunabot_msgs/msg/motor_commands.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include <chrono>
#include <thread>

/**
 * @class NavigationClient
 * @brief Runs autonomous one cycle sequence: navigation, excavation, and depositing actions.
 */
class NavigationClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using Excavation = lunabot_msgs::action::Excavation;
  using GoalHandleExcavation = rclcpp_action::ClientGoalHandle<Excavation>;
  using Depositing = lunabot_msgs::action::Depositing;
  using GoalHandleDepositing = rclcpp_action::ClientGoalHandle<Depositing>;

  enum class State {
    IDLE,
    DRIVING_FROM_START,
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
    current_state_ = State::DRIVING_FROM_START;

    // Declare parameter for using light excavation (default: true for autonomous)
    this->declare_parameter("use_light_excavation", true);
    use_light_excavation_ = this->get_parameter("use_light_excavation").as_bool();

    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

    // Connect to appropriate excavation action based on parameter
    std::string excavation_action_name =
      use_light_excavation_ ? "light_excavation_action" : "excavation_action";

    excavation_client_ = rclcpp_action::create_client<Excavation>(this, excavation_action_name);
    depositing_client_ = rclcpp_action::create_client<Depositing>(this, "depositing_action");
    motor_cmd_publisher_ =
      this->create_publisher<lunabot_msgs::msg::MotorCommands>("/motor_commands", 10);

    if (use_light_excavation_)
    {
      LOGGER_INFO(this->get_logger(), "Using LIGHT excavation mode (minimal vibration)");
    } else
    {
      LOGGER_INFO(this->get_logger(), "Using STANDARD excavation mode");
    }

    execution_timer_ =
      this->create_wall_timer(std::chrono::seconds(1), std::bind(&NavigationClient::execute, this));
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
      case State::DRIVING_FROM_START:
        drive_forward_from_start();
        current_state_ = State::EXCAVATING;
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
        LOGGER_SUCCESS(this->get_logger(), "Cycle complete!");
        execution_timer_->cancel();
        break;
    }
  }

  /**
   * @brief Drives robot forward for 3 seconds from starting position.
   */
  void drive_forward_from_start()
  {
    LOGGER_ACTION(this->get_logger(), "Driving forward from start for 3 seconds...");

    auto motor_cmd = lunabot_msgs::msg::MotorCommands();
    motor_cmd.left_actuator = 0.0;
    motor_cmd.right_actuator = 0.0;
    motor_cmd.vibration = 0.0;

    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(3))
    {
      motor_cmd.left_wheel = 0.5;
      motor_cmd.right_wheel = -0.5;
      motor_cmd_publisher_->publish(motor_cmd);

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    motor_cmd.left_wheel = 0.0;
    motor_cmd.right_wheel = 0.0;
    motor_cmd_publisher_->publish(motor_cmd);

    LOGGER_SUCCESS(this->get_logger(), "Forward drive complete. Ready for excavation.");
  }

  /**
   * @brief Sends a request to the excavation server.
   */
  void request_excavation()
  {
    if (!excavation_client_->wait_for_action_server(std::chrono::seconds(1)))
    {
      LOGGER_WARN_ONCE(this->get_logger(), "Excavation action server not available.");
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
  void handle_excavation_result(const GoalHandleExcavation::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      LOGGER_SUCCESS(this->get_logger(), "Excavation success! Navigating to construction zone...");
      current_state_ = State::NAVIGATING_TO_CONSTRUCTION;
    } else
    {
      LOGGER_FAILURE(this->get_logger(), "Excavation failed.");
      current_state_ = State::IDLE;
    }
  }

  /**
   * @brief Sends goal request to navigation server for construction zone.
   */
  void request_navigation_to_construction()
  {
    // Wait for Nav2 action server to be ready
    LOGGER_INFO(this->get_logger(), "Waiting for Nav2 action server to be ready...");
    if (!navigation_client_->wait_for_action_server(std::chrono::seconds(30)))
    {
      LOGGER_FAILURE(this->get_logger(), "Navigation action server not available after 30s.");
      return;
    }

    LOGGER_INFO(this->get_logger(), "Nav2 action server ready!");

    auto goal_msg = NavigateToPose::Goal();
    geometry_msgs::msg::Pose goal_pose;

    // Navigate to construction zone coordinates
    goal_pose.position.x = -4.5;
    goal_pose.position.y = -0.7;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = -0.7071068;
    goal_pose.orientation.w = 0.7071068;

    goal_msg.pose.pose = goal_pose;
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = "map";

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(
      &NavigationClient::handle_construction_navigation_result, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(
      &NavigationClient::handle_construction_feedback, this, std::placeholders::_1,
      std::placeholders::_2);

    LOGGER_ACTION(
      this->get_logger(), "Sending navigation goal to construction zone [-4.5, 0.4]...");
    navigation_client_->async_send_goal(goal_msg, send_goal_options);
  }

  /**
   * @brief Callback for navigation feedback to construction zone.
   * @param goal_handle The goal handle.
   * @param feedback The feedback message containing distance remaining.
   */
  void handle_construction_feedback(
    GoalHandleNavigate::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    LOGGER_INFO(
      this->get_logger(), "Distance to construction zone: %.2f meters",
      feedback->distance_remaining);
  }

  /**
   * @brief Callback for the result of navigation to construction zone.
   * @param result The result of the goal execution.
   */
  void handle_construction_navigation_result(const GoalHandleNavigate::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      LOGGER_SUCCESS(this->get_logger(), "Construction zone reached. Requesting deposit...");
      current_state_ = State::DEPOSITING;
    } else if (result.code == rclcpp_action::ResultCode::ABORTED)
    {
      LOGGER_FAILURE(this->get_logger(), "Goal aborted, unable to reach construction zone");
      current_state_ = State::IDLE;
    } else if (result.code == rclcpp_action::ResultCode::CANCELED)
    {
      LOGGER_WARN(this->get_logger(), "Navigation to construction zone canceled");
      current_state_ = State::IDLE;
    } else
    {
      LOGGER_FAILURE(
        this->get_logger(), "Navigation to construction zone failed with unknown result");
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
      LOGGER_WARN_ONCE(this->get_logger(), "Depositing action server not available.");
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
  void handle_depositing_result(const GoalHandleDepositing::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      LOGGER_SUCCESS(this->get_logger(), "Depositing success! Cycle complete!");
      current_state_ = State::COMPLETE;
    } else
    {
      LOGGER_FAILURE(this->get_logger(), "Depositing failed.");
      current_state_ = State::IDLE;
    }
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp_action::Client<Excavation>::SharedPtr excavation_client_;
  rclcpp_action::Client<Depositing>::SharedPtr depositing_client_;
  rclcpp::Publisher<lunabot_msgs::msg::MotorCommands>::SharedPtr motor_cmd_publisher_;
  rclcpp::TimerBase::SharedPtr execution_timer_;

  State current_state_;
  bool use_light_excavation_;
};

/**
 * @brief Main function.
 * Initializes and runs the NavigationClient node.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationClient>());
  rclcpp::shutdown();
  return 0;
}
