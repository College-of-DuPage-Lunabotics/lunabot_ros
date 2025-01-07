/**
 * @file excavation_server.cpp
 * @author Grayson Arendt
 * @date 12/29/2024
 */

#include <chrono>
#include <cmath>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "lunabot_msgs/action/excavation.hpp"

/**
 * @class ExcavationServer
 * @brief Performs excavation sequence after receiving request from the navigation client.
 */
class ExcavationServer : public rclcpp::Node
{
  public:
    using Excavation = lunabot_msgs::action::Excavation;
    using GoalHandleExcavation = rclcpp_action::ServerGoalHandle<Excavation>;

    /**
     * @brief Constructor for the ExcavationServer class.
     */
    ExcavationServer()
        : Node("excavation_server"), success_(false), goal_active_(false), alignment_done_(false), current_x_(0.0),
          current_y_(0.0), current_yaw_(0.0), previous_error_(0.0), error_sum_(0.0), previous_time_(this->now())
    {
        this->declare_parameter("kP", 7.0);  // Proportional gain
        this->declare_parameter("kI", 0.15); // Integral gain
        this->declare_parameter("kD", 3.0);  // Derivative gain

        cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        action_server_ = rclcpp_action::create_server<Excavation>(
            this, "excavation_action",
            std::bind(&ExcavationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ExcavationServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ExcavationServer::handle_accepted, this, std::placeholders::_1));

        odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
            "icp_odom", 10, std::bind(&ExcavationServer::odometry_callback, this, std::placeholders::_1));

        excavation_timer_ =
            create_wall_timer(std::chrono::milliseconds(100), std::bind(&ExcavationServer::excavate, this));
    }

  private:
    /**
     * @brief Handles goal requests from clients.
     * @return Goal response (accept or reject).
     */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Excavation::Goal>)
    {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * @brief Handles cancellation requests from clients.
     * @return Cancel response (accept or reject).
     */
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExcavation>)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     * @brief Accepts and starts the goal execution.
     * @param goal_handle Handle for the current goal.
     */
    void handle_accepted(const std::shared_ptr<GoalHandleExcavation> goal_handle)
    {
        goal_active_ = true;
        std::thread{std::bind(&ExcavationServer::execute, this, goal_handle)}.detach();
    }

    /**
     * @brief Executes the Excavation process.
     * @param goal_handle Handle for the current goal.
     */
    void execute(const std::shared_ptr<GoalHandleExcavation> goal_handle)
    {
        auto result = std::make_shared<Excavation::Result>();

        while (!success_ && rclcpp::ok())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (success_)
        {
            result->success = true;
            goal_handle->succeed(result);
            rclcpp::shutdown();
        }
    }

    /**
     * @brief Callback for the odometry message.
     * @param msg The odometry message.
     */
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        tf2::Quaternion quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 matrix(quaternion);

        matrix.getRPY(current_roll_, current_pitch_, current_yaw_);
    }

    /**
     * @brief Uses current odometry and compares to goal to navigate the robot to
     * the construction zone.
     */
    void excavate()
    {
        if (!goal_active_)
        {
            return;
        }

        double kP = this->get_parameter("kP").as_double();
        double kI = this->get_parameter("kI").as_double();
        double kD = this->get_parameter("kD").as_double();

        rclcpp::Time current_time = this->now();
        double dt = (current_time - previous_time_).seconds();

        double target_x = 4.3;
        double target_y = -0.2;

        double angle_to_goal = atan2(target_y - current_y_, target_x - current_x_);
        double yaw_error = normalize_angle(angle_to_goal + M_PI - current_yaw_);

        auto twist_msg = geometry_msgs::msg::Twist();

        if (!alignment_done_)
        {
            if (std::abs(yaw_error) > 0.05)
            {
                twist_msg.angular.z = 0.3 * yaw_error / std::abs(yaw_error);
                twist_msg.linear.x = 0.0;
                cmd_vel_publisher_->publish(twist_msg);
                RCLCPP_INFO(this->get_logger(), "\033[1;34mALIGNING TO GOAL... YAW: %.2f\033[0m", current_yaw_);
            }
            else
            {
                alignment_done_ = true;
                RCLCPP_INFO(this->get_logger(), "\033[1;36mALIGNMENT COMPLETE, STARTING BACKWARD MOVEMENT...\033[0m");
            }
            return;
        }

        error_sum_ += yaw_error * dt;
        double predicted_error = (yaw_error - previous_error_) / dt;

        twist_msg.linear.x = -0.3;
        twist_msg.angular.z = kP * yaw_error + kI * error_sum_ + kD * predicted_error;

        previous_error_ = yaw_error;
        previous_time_ = current_time;

        double distance_to_goal = sqrt(pow(target_x - current_x_, 2) + pow(target_y - current_y_, 2));
        RCLCPP_INFO(this->get_logger(), "\033[1;35mDISTANCE TO GOAL: %.2f METERS, YAW ERROR: %.2f\033[0m",
                    distance_to_goal, yaw_error);

        if (distance_to_goal <= 0.3)
        {
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            success_ = true;
        }

        cmd_vel_publisher_->publish(twist_msg);
    }

    /**
     * @brief Normalizes an angle to the range [-pi, pi].
     * @param angle The angle to normalize.
     * @return Normalized angle.
     */
    double normalize_angle(double angle)
    {
        while (angle > M_PI)
            angle -= 2.0 * M_PI;
        while (angle < -M_PI)
            angle += 2.0 * M_PI;
        return angle;
    }

    rclcpp_action::Server<Excavation>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::TimerBase::SharedPtr odometry_timer_, excavation_timer_;
    rclcpp::Time previous_time_;

    bool success_, goal_active_, alignment_done_;
    double current_x_, current_y_, current_roll_, current_pitch_, current_yaw_;
    double previous_error_, error_sum_;
};

/**
 * @brief Main function.
 * Initializes and runs the ExcavationServer node.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ExcavationServer>());
    rclcpp::shutdown();
    return 0;
}