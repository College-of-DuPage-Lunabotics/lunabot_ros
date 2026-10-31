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
#include "nav2_msgs/action/drive_on_heading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2_ros/buffer.h"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_listener.h"

#include <chrono>
#include <cmath>
#include <thread>

enum class CompetitionMode { KSC, UCF };

static constexpr double rotation_speed = 0.15;
static constexpr int rotation_90_deg_seconds = 10;

static constexpr int feedback_log_period_ms = 5000;

static constexpr double first_intermediate_waypoint_x = 0.0;
static constexpr double first_intermediate_waypoint_y = 3.0;

static constexpr double intermediate_waypoint_x = 4.8;
static constexpr double intermediate_waypoint_y = 3.0;

// Drive out of the dig, lidar first
static constexpr double ucf_backup_distance = 1.0;
static constexpr double backup_speed = 0.1;
static constexpr int backup_time_allowance_seconds = 60;

// In place turns, below 0.08 rad/s the wheels stall
static constexpr double spin_max_speed = 0.15;
static constexpr double spin_min_speed = 0.08;
static constexpr double spin_accel = 0.1;
static constexpr double spin_slowdown_gain = 0.6;
static constexpr double spin_done_tolerance = 0.035;
static constexpr double spin_loop_hz = 20.0;

static constexpr double dig_heading_tolerance = 15.0 * M_PI / 180.0;

static constexpr int ucf_map_wait_seconds = 30;

static constexpr double ksc_construction_zone_x = 4.5;
static constexpr double ksc_construction_zone_y = 0.4;

// UCF arena frame: origin at the bottom wall center, x right, y up
static constexpr double ucf_berm_x = 3.56;
static constexpr double ucf_berm_y = 6.4;

// base_link offset from the berm center so the bucket pivot (0.2 m ahead) dumps mid berm
static constexpr double ucf_deposit_offset_x = -0.15;

// Start pose in the arena frame, lidar facing east
static constexpr double ucf_start_x = -1.7;
static constexpr double ucf_start_y = 1.55;
static constexpr double ucf_start_yaw = 0.0;

// Lidar headings in the map frame
static constexpr double ucf_mapping_spin_yaw = M_PI - ucf_start_yaw;
static constexpr double ucf_spin_yaw = M_PI / 2.0 - ucf_start_yaw;

/**
 * @class NavigationClient
 * @brief Runs autonomous one cycle sequence: navigation, excavation, and depositing actions.
 */
class NavigationClient : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using DriveOnHeading = nav2_msgs::action::DriveOnHeading;
  using GoalHandleDriveOnHeading = rclcpp_action::ClientGoalHandle<DriveOnHeading>;
  using Excavation = lunabot_msgs::action::Excavation;
  using GoalHandleExcavation = rclcpp_action::ClientGoalHandle<Excavation>;
  using Depositing = lunabot_msgs::action::Depositing;
  using GoalHandleDepositing = rclcpp_action::ClientGoalHandle<Depositing>;

  enum class State {
    IDLE,
    WAITING_FOR_MAP,
    MAPPING_SPIN,
    FREEZING_MAP,
    EXCAVATING,
    BACKING_UP,
    SPINNING,
    NAVIGATING_TO_FIRST_INTERMEDIATE,
    ROTATING_AFTER_FIRST_INTERMEDIATE,
    NAVIGATING_TO_INTERMEDIATE,
    NAVIGATING_TO_CONSTRUCTION,
    DEPOSITING,
    COMPLETE
  };

  /**
   * @brief Constructor for the NavigationClient class.
   */
  NavigationClient() : Node("navigation_client")
  {
    this->declare_parameter("mode", "ucf");
    std::string mode_str = this->get_parameter("mode").as_string();
    // Drive only, the bucket never moves
    dry_run_ = this->declare_parameter("dry_run", false);

    if (mode_str == "ucf")
    {
      mode_ = CompetitionMode::UCF;
      current_state_ = State::WAITING_FOR_MAP;
      LOGGER_INFO(this->get_logger(), "UCF Mode");
    } else
    {
      mode_ = CompetitionMode::KSC;
      current_state_ = State::NAVIGATING_TO_FIRST_INTERMEDIATE;
      LOGGER_INFO(this->get_logger(), "KSC Mode");
    }
    if (dry_run_)
    {
      LOGGER_WARN(this->get_logger(), "Dry run: driving only, the bucket never moves");
    }

    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    backup_client_ = rclcpp_action::create_client<DriveOnHeading>(this, "drive_on_heading");
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pause_map_client_ = this->create_client<std_srvs::srv::Empty>("/rtabmap/pause");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    excavation_client_ = rclcpp_action::create_client<Excavation>(this, "auto_excavation_action");
    depositing_client_ = rclcpp_action::create_client<Depositing>(this, "auto_depositing_action");
    motor_cmd_publisher_ =
      this->create_publisher<lunabot_msgs::msg::MotorCommands>("/motor_commands", 10);

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
      case State::WAITING_FOR_MAP:
        wait_for_map();
        break;
      case State::MAPPING_SPIN:
        current_state_ = State::IDLE;
        request_spin(ucf_mapping_spin_yaw, State::FREEZING_MAP, true);
        break;
      case State::FREEZING_MAP:
        freeze_map();
        current_state_ = State::EXCAVATING;
        break;
      case State::EXCAVATING:
        if (mode_ == CompetitionMode::UCF && !facing_dig_direction())
        {
          break;
        }
        request_excavation();
        current_state_ = State::IDLE;
        break;
      case State::BACKING_UP:
        request_backup();
        current_state_ = State::IDLE;
        break;
      case State::SPINNING:
        current_state_ = State::IDLE;
        request_spin(ucf_spin_yaw, State::NAVIGATING_TO_CONSTRUCTION);
        break;
      case State::NAVIGATING_TO_FIRST_INTERMEDIATE:
        request_navigation_to_first_intermediate();
        current_state_ = State::IDLE;
        break;
      case State::ROTATING_AFTER_FIRST_INTERMEDIATE:
        rotate_90_degrees();
        current_state_ = State::NAVIGATING_TO_INTERMEDIATE;
        break;
      case State::NAVIGATING_TO_INTERMEDIATE:
        request_navigation_to_intermediate();
        current_state_ = State::IDLE;
        break;
      case State::NAVIGATING_TO_CONSTRUCTION:
        request_navigation_to_construction();
        current_state_ = State::IDLE;
        break;
      case State::DEPOSITING:
        if (dry_run_)
        {
          LOGGER_INFO(this->get_logger(), "Dry run, skipping depositing");
          current_state_ = State::COMPLETE;
          break;
        }
        request_depositing();
        current_state_ = State::IDLE;
        break;
      case State::COMPLETE:
        LOGGER_SUCCESS(this->get_logger(), "Cycle complete!");
        execution_timer_->cancel();
        break;
    }
  }

  void rotate_90_degrees()
  {
    LOGGER_ACTION(
      this->get_logger(), "Rotating 90 degrees to the right for %d seconds...",
      rotation_90_deg_seconds);

    auto motor_cmd = lunabot_msgs::msg::MotorCommands();
    motor_cmd.left_actuator = 0.0;
    motor_cmd.right_actuator = 0.0;
    motor_cmd.vibration = 0.0;

    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time <
           std::chrono::seconds(rotation_90_deg_seconds))
    {
      motor_cmd.left_wheel = -rotation_speed;
      motor_cmd.right_wheel = -rotation_speed;
      motor_cmd_publisher_->publish(motor_cmd);

      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    motor_cmd.left_wheel = 0.0;
    motor_cmd.right_wheel = 0.0;
    motor_cmd_publisher_->publish(motor_cmd);

    LOGGER_SUCCESS(this->get_logger(), "Rotation complete. Ready for navigation.");
  }

  /**
   * @brief Sends a request to the excavation server.
   */
  void request_excavation()
  {
    if (!excavation_client_->wait_for_action_server(std::chrono::seconds(30)))
    {
      LOGGER_WARN_ONCE(this->get_logger(), "Excavation action server not available.");
      return;
    }

    auto goal_msg = Excavation::Goal();
    goal_msg.drive_only = dry_run_;
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
      LOGGER_SUCCESS(this->get_logger(), "Excavation success! Driving out of the dig...");
      current_state_ = State::BACKING_UP;
    } else
    {
      LOGGER_FAILURE(this->get_logger(), "Excavation failed.");
      current_state_ = State::IDLE;
    }
  }

  void request_navigation_to_first_intermediate()
  {
    LOGGER_INFO(this->get_logger(), "Waiting for Nav2 action server to be ready...");
    if (!navigation_client_->wait_for_action_server(std::chrono::seconds(30)))
    {
      LOGGER_FAILURE(this->get_logger(), "Navigation action server not available after 30s.");
      return;
    }

    LOGGER_INFO(this->get_logger(), "Nav2 action server ready!");

    auto goal_msg = NavigateToPose::Goal();
    geometry_msgs::msg::Pose goal_pose;

    goal_pose.position.x = first_intermediate_waypoint_x;
    goal_pose.position.y = first_intermediate_waypoint_y;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.0;
    goal_pose.orientation.w = 1.0;

    goal_msg.pose.pose = goal_pose;
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = "map";

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(
      &NavigationClient::handle_first_intermediate_navigation_result, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(
      &NavigationClient::handle_first_intermediate_feedback, this, std::placeholders::_1,
      std::placeholders::_2);

    LOGGER_ACTION(
      this->get_logger(), "Sending navigation goal to first intermediate waypoint [%.1f, %.1f]...",
      first_intermediate_waypoint_x, first_intermediate_waypoint_y);
    navigation_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void handle_first_intermediate_feedback(
    GoalHandleNavigate::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    LOGGER_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), feedback_log_period_ms,
      "Distance to first intermediate waypoint: %.2f meters", feedback->distance_remaining);
  }

  void handle_first_intermediate_navigation_result(const GoalHandleNavigate::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      LOGGER_SUCCESS(
        this->get_logger(), "First intermediate waypoint reached. Rotating 90 degrees...");
      current_state_ = State::ROTATING_AFTER_FIRST_INTERMEDIATE;
    } else if (result.code == rclcpp_action::ResultCode::ABORTED)
    {
      LOGGER_WARN(this->get_logger(), "First intermediate waypoint aborted, rotating anyway...");
      current_state_ = State::ROTATING_AFTER_FIRST_INTERMEDIATE;
    } else if (result.code == rclcpp_action::ResultCode::CANCELED)
    {
      LOGGER_WARN(this->get_logger(), "First intermediate waypoint canceled, rotating anyway...");
      current_state_ = State::ROTATING_AFTER_FIRST_INTERMEDIATE;
    } else
    {
      LOGGER_WARN(this->get_logger(), "First intermediate waypoint failed, rotating anyway...");
      current_state_ = State::ROTATING_AFTER_FIRST_INTERMEDIATE;
    }
  }

  void request_navigation_to_intermediate()
  {
    LOGGER_INFO(this->get_logger(), "Waiting for Nav2 action server to be ready...");
    if (!navigation_client_->wait_for_action_server(std::chrono::seconds(30)))
    {
      LOGGER_FAILURE(this->get_logger(), "Navigation action server not available after 30s.");
      return;
    }

    LOGGER_INFO(this->get_logger(), "Nav2 action server ready!");

    auto goal_msg = NavigateToPose::Goal();
    geometry_msgs::msg::Pose goal_pose;

    goal_pose.position.x = intermediate_waypoint_x;
    goal_pose.position.y = intermediate_waypoint_y;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 0.7071068;
    goal_pose.orientation.w = 0.7071068;

    goal_msg.pose.pose = goal_pose;
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.header.frame_id = "map";

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(
      &NavigationClient::handle_intermediate_navigation_result, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(
      &NavigationClient::handle_intermediate_feedback, this, std::placeholders::_1,
      std::placeholders::_2);

    LOGGER_ACTION(
      this->get_logger(), "Sending navigation goal to intermediate waypoint [%.1f, %.1f]...",
      intermediate_waypoint_x, intermediate_waypoint_y);
    navigation_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void handle_intermediate_feedback(
    GoalHandleNavigate::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    LOGGER_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), feedback_log_period_ms,
      "Distance to intermediate waypoint: %.2f meters", feedback->distance_remaining);
  }

  void handle_intermediate_navigation_result(const GoalHandleNavigate::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      LOGGER_SUCCESS(this->get_logger(), "Intermediate waypoint reached. Navigating to berm...");
      current_state_ = State::NAVIGATING_TO_CONSTRUCTION;
    } else if (result.code == rclcpp_action::ResultCode::ABORTED)
    {
      LOGGER_WARN(
        this->get_logger(), "Intermediate waypoint aborted, proceeding to berm anyway...");
      current_state_ = State::NAVIGATING_TO_CONSTRUCTION;
    } else if (result.code == rclcpp_action::ResultCode::CANCELED)
    {
      LOGGER_WARN(
        this->get_logger(), "Intermediate waypoint canceled, proceeding to berm anyway...");
      current_state_ = State::NAVIGATING_TO_CONSTRUCTION;
    } else
    {
      LOGGER_WARN(this->get_logger(), "Intermediate waypoint failed, proceeding to berm anyway...");
      current_state_ = State::NAVIGATING_TO_CONSTRUCTION;
    }
  }

  /**
   * @brief Drives straight forward out of the excavation path, undoing the backwards dig.
   */
  void request_backup()
  {
    if (!backup_client_->wait_for_action_server(std::chrono::seconds(30)))
    {
      LOGGER_FAILURE(this->get_logger(), "Drive on heading action server not available after 30s.");
      return;
    }

    auto goal_msg = DriveOnHeading::Goal();
    goal_msg.target.x = ucf_backup_distance;
    goal_msg.speed = backup_speed;
    goal_msg.time_allowance.sec = backup_time_allowance_seconds;

    auto send_goal_options = rclcpp_action::Client<DriveOnHeading>::SendGoalOptions();
    send_goal_options.result_callback =
      std::bind(&NavigationClient::handle_backup_result, this, std::placeholders::_1);

    LOGGER_ACTION(this->get_logger(), "Driving forward %.2f meters out of the dig...", ucf_backup_distance);
    backup_client_->async_send_goal(goal_msg, send_goal_options);
  }

  /**
   * @brief Callback for the result of the backup, moves on to the berm either way.
   * @param result The result of the backup action.
   */
  void handle_backup_result(const GoalHandleDriveOnHeading::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      LOGGER_SUCCESS(this->get_logger(), "Drive out complete. Spinning...");
    } else
    {
      LOGGER_WARN(this->get_logger(), "Drive out did not finish, spinning anyway...");
    }
    current_state_ = State::SPINNING;
  }

  /**
   * @brief Turns in place so the robot faces target_yaw, then moves on to next_state.
   * @param target_yaw Map yaw to face.
   * @param next_state State to run once the turn finishes or fails.
   * @param counter_clockwise Always turn counter clockwise, otherwise the short way.
   */
  void request_spin(double target_yaw, State next_state, bool counter_clockwise = false)
  {
    rotate_to(target_yaw, counter_clockwise);
    current_state_ = next_state;
  }

  /**
   * @brief Turns in place with cmd_vel, closed loop on the TF heading. Blocks until done or timed out.
   * @param target_yaw Map yaw to face.
   * @param counter_clockwise Always turn counter clockwise, otherwise the short way.
   * @return True if the target heading was reached.
   */
  bool rotate_to(double target_yaw, bool counter_clockwise)
  {
    double yaw;
    if (!get_yaw(yaw))
    {
      LOGGER_WARN(this->get_logger(), "No robot heading available, skipping turn");
      return false;
    }

    double turn = std::atan2(std::sin(target_yaw - yaw), std::cos(target_yaw - yaw));
    if (counter_clockwise && turn < -spin_done_tolerance)
    {
      turn += 2.0 * M_PI;
    }
    if (std::fabs(turn) < spin_done_tolerance)
    {
      LOGGER_SUCCESS(this->get_logger(), "Already facing the target heading.");
      return true;
    }

    LOGGER_ACTION(this->get_logger(), "Turning %.0f degrees...", turn * 180.0 / M_PI);
    const double direction = turn > 0.0 ? 1.0 : -1.0;
    const auto timeout = std::chrono::duration<double>(2.0 * std::fabs(turn) / spin_min_speed + 10.0);
    const auto start_time = std::chrono::steady_clock::now();

    double turned = 0.0;
    double last_yaw = yaw;
    double speed = 0.0;
    bool reached = false;
    geometry_msgs::msg::Twist cmd;
    rclcpp::Rate rate(spin_loop_hz);

    while (rclcpp::ok())
    {
      if (std::chrono::steady_clock::now() - start_time > timeout)
      {
        break;
      }
      if (get_yaw(yaw))
      {
        turned += std::atan2(std::sin(yaw - last_yaw), std::cos(yaw - last_yaw));
        last_yaw = yaw;
      }
      double left = std::fabs(turn) - direction * turned;
      if (left < spin_done_tolerance)
      {
        reached = true;
        break;
      }

      speed = std::min(
        {speed + spin_accel / spin_loop_hz, spin_max_speed,
         std::max(spin_min_speed, spin_slowdown_gain * left)});
      cmd.angular.z = direction * speed;
      cmd_vel_publisher_->publish(cmd);
      rate.sleep();
    }

    cmd.angular.z = 0.0;
    cmd_vel_publisher_->publish(cmd);

    if (reached)
    {
      LOGGER_SUCCESS(this->get_logger(), "Turn complete.");
    } else
    {
      LOGGER_WARN(
        this->get_logger(), "Turn timed out after %.0f of %.0f degrees", direction * turned * 180.0 / M_PI,
        std::fabs(turn) * 180.0 / M_PI);
    }
    return reached;
  }

  /**
   * @brief Checks the bucket faces the dig direction, re-spins once if not, otherwise stops.
   * @return True if it is safe to dig.
   */
  bool facing_dig_direction()
  {
    double yaw;
    if (!get_yaw(yaw))
    {
      LOGGER_FAILURE(this->get_logger(), "No robot heading available, not digging. Cycle stopped.");
      current_state_ = State::IDLE;
      return false;
    }

    double error = std::atan2(std::sin(ucf_mapping_spin_yaw - yaw), std::cos(ucf_mapping_spin_yaw - yaw));
    if (std::fabs(error) <= dig_heading_tolerance)
    {
      return true;
    }

    if (!dig_heading_corrected_)
    {
      dig_heading_corrected_ = true;
      LOGGER_WARN(
        this->get_logger(), "Off the dig heading by %.0f deg, turning before digging...",
        error * 180.0 / M_PI);
      current_state_ = State::IDLE;
      request_spin(ucf_mapping_spin_yaw, State::EXCAVATING);
      return false;
    }

    LOGGER_FAILURE(
      this->get_logger(), "Still off the dig heading by %.0f deg, not digging into the wall. "
      "Cycle stopped.", error * 180.0 / M_PI);
    current_state_ = State::IDLE;
    return false;
  }

  /**
   * @brief Robot yaw in the map frame, or in odom if RTAB-Map is not running (they start equal).
   * @param yaw Output yaw.
   * @return True if either transform was available.
   */
  bool get_yaw(double & yaw)
  {
    if (get_map_yaw(yaw))
    {
      return true;
    }
    try
    {
      auto t = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
      const auto & q = t.transform.rotation;
      yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      return true;
    } catch (const tf2::TransformException &)
    {
      return false;
    }
  }

  /**
   * @brief Looks up the robot yaw in the map frame.
   * @param yaw Output map yaw.
   * @return True if the transform was available.
   */
  bool get_map_yaw(double & yaw)
  {
    try
    {
      auto t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
      const auto & q = t.transform.rotation;
      yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      return true;
    } catch (const tf2::TransformException &)
    {
      return false;
    }
  }

  /**
   * @brief Waits for RTAB-Map's map frame before the mapping spin, gives up after a timeout.
   */
  void wait_for_map()
  {
    double yaw;
    if (get_map_yaw(yaw))
    {
      LOGGER_SUCCESS(this->get_logger(), "Map frame available. Mapping spin...");
      current_state_ = State::MAPPING_SPIN;
      return;
    }

    if (++map_wait_seconds_ >= ucf_map_wait_seconds)
    {
      LOGGER_FAILURE(
        this->get_logger(), "No map frame after %d s, RTAB-Map is not running. Skipping mapping, "
        "navigation will fail without it", ucf_map_wait_seconds);
      current_state_ = State::EXCAVATING;
      return;
    }

    LOGGER_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Waiting for RTAB-Map map frame...");
  }

  /**
   * @brief Pauses RTAB-Map so the start map stops changing.
   */
  void freeze_map()
  {
    if (!pause_map_client_->wait_for_service(std::chrono::seconds(2)))
    {
      LOGGER_WARN(this->get_logger(), "RTAB-Map pause service not available, map keeps updating");
      return;
    }

    pause_map_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
    LOGGER_SUCCESS(this->get_logger(), "Map frozen, RTAB-Map paused. Excavating...");
  }

  /**
   * @brief Converts a UCF arena point to the map frame, which starts at the robot start pose.
   * @param arena_x Arena x (right of the bottom wall center).
   * @param arena_y Arena y (up from the bottom wall).
   * @param map_x Output map x.
   * @param map_y Output map y.
   */
  void arena_to_map(double arena_x, double arena_y, double & map_x, double & map_y)
  {
    double dx = arena_x - ucf_start_x;
    double dy = arena_y - ucf_start_y;
    map_x = std::cos(ucf_start_yaw) * dx + std::sin(ucf_start_yaw) * dy;
    map_y = -std::sin(ucf_start_yaw) * dx + std::cos(ucf_start_yaw) * dy;
  }

  /**
   * @brief Sends goal request to navigation server for construction zone.
   */
  void request_navigation_to_construction()
  {
    LOGGER_INFO(this->get_logger(), "Waiting for Nav2 action server to be ready...");
    if (!navigation_client_->wait_for_action_server(std::chrono::seconds(30)))
    {
      LOGGER_FAILURE(this->get_logger(), "Navigation action server not available after 30s.");
      return;
    }

    LOGGER_INFO(this->get_logger(), "Nav2 action server ready!");

    auto goal_msg = NavigateToPose::Goal();
    geometry_msgs::msg::Pose goal_pose;

    double construction_zone_x, construction_zone_y;
    if (mode_ == CompetitionMode::UCF)
    {
      arena_to_map(
        ucf_berm_x + ucf_deposit_offset_x, ucf_berm_y, construction_zone_x, construction_zone_y);
    } else
    {
      construction_zone_x = ksc_construction_zone_x;
      construction_zone_y = ksc_construction_zone_y;
    }

    goal_pose.position.x = construction_zone_x;
    goal_pose.position.y = construction_zone_y;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 0.0;
    goal_pose.orientation.z = 1.0;
    goal_pose.orientation.w = 0.0;

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
      this->get_logger(), "Sending navigation goal to berm zone [%.1f, %.1f]...",
      construction_zone_x, construction_zone_y);
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
    LOGGER_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), feedback_log_period_ms,
      "Distance to berm zone: %.2f meters", feedback->distance_remaining);
  }

  /**
   * @brief Callback for the result of navigation to construction zone.
   * @param result The result of the goal execution.
   */
  void handle_construction_navigation_result(const GoalHandleNavigate::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      LOGGER_SUCCESS(this->get_logger(), "Berm zone reached. Requesting deposit...");
      current_state_ = State::DEPOSITING;
    } else if (result.code == rclcpp_action::ResultCode::ABORTED)
    {
      LOGGER_FAILURE(this->get_logger(), "Goal aborted, unable to reach berm zone");
      current_state_ = State::IDLE;
    } else if (result.code == rclcpp_action::ResultCode::CANCELED)
    {
      LOGGER_WARN(this->get_logger(), "Navigation to berm zone canceled");
      current_state_ = State::IDLE;
    } else
    {
      LOGGER_FAILURE(this->get_logger(), "Navigation to berm zone failed with unknown result");
      current_state_ = State::IDLE;
    }
  }

  /**
   * @brief Sends a request to the depositing server.
   */
  void request_depositing()
  {
    if (!depositing_client_->wait_for_action_server(std::chrono::seconds(30)))
    {
      LOGGER_WARN_ONCE(this->get_logger(), "Depositing action server not available.");
      return;
    }

    auto goal_msg = Depositing::Goal();
    goal_msg.dump_only = true;
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
  rclcpp_action::Client<DriveOnHeading>::SharedPtr backup_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr pause_map_client_;
  int map_wait_seconds_ = 0;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<Excavation>::SharedPtr excavation_client_;
  rclcpp_action::Client<Depositing>::SharedPtr depositing_client_;
  rclcpp::Publisher<lunabot_msgs::msg::MotorCommands>::SharedPtr motor_cmd_publisher_;
  rclcpp::TimerBase::SharedPtr execution_timer_;

  State current_state_;
  bool dig_heading_corrected_ = false;
  bool dry_run_ = false;
  CompetitionMode mode_;
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
