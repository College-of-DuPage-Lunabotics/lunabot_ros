/**
 * @file navigation_client.cpp
 * @author Grayson Arendt
 * @date 11/11/2024
 */

#include <chrono>
#include <cmath>

#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "lunabot_system/action/localization.hpp"

/**
 * @class NavigationClient
 * @brief Sends navigation goals, handles received localization data, and controls robot movement based on odometry
 * feedback.
 */
class NavigationClient : public rclcpp::Node
{
  public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using Localization = lunabot_system::action::Localization;
    using GoalHandleLocalization = rclcpp_action::ClientGoalHandle<Localization>;

    /**
     * @brief Constructor for the NavigationClient class.
     */
    NavigationClient() : Node("navigator_client"), goal_reached_(false), localization_done_(false)
    {
        nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        localization_client_ = rclcpp_action::create_client<Localization>(this, "localization_action");
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/filtered", 10, std::bind(&NavigationClient::odom_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&NavigationClient::localize_and_send_goal, this));
    }

  private:
    /**
     * @brief Calls the localization action and starts the navigation to the first goal.
     */
    void localize_and_send_goal()
    {
        if (!localization_done_)
        {
            if (!localization_client_->wait_for_action_server(std::chrono::seconds(10)))
            {
                RCLCPP_ERROR(this->get_logger(), "\033[1;31mLOCALIZATION ACTION SERVER NOT AVAILABLE.\033[0m");
                rclcpp::shutdown();
                return;
            }

            auto goal_msg = Localization::Goal();
            auto send_goal_options = rclcpp_action::Client<Localization>::SendGoalOptions();
            send_goal_options.result_callback =
                std::bind(&NavigationClient::handle_localization_result, this, std::placeholders::_1);
            localization_client_->async_send_goal(goal_msg, send_goal_options);
        }

        if (!goal_reached_)
        {
            send_first_goal();
        }
    }

    /**
     * @brief Handles the result from the localization action.
     * @param result The result from the localization action.
     */
    void handle_localization_result(const GoalHandleLocalization::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            initial_x_ = result.result->x;
            initial_y_ = result.result->y;
            localization_done_ = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "\033[1;31mLOCALIZATION FAILED.\033[0m");
            rclcpp::shutdown();
        }
    }

    /**
     * @brief Sends the first navigation goal.
     */
    void send_first_goal()
    {
        if (!this->nav_to_pose_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "\033[1;31mNAV2 ACTION SERVER NOT AVAILABLE.\033[0m");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        geometry_msgs::msg::Pose goal_pose;

        goal_pose.position.x = initial_x_ + 3.7;
        goal_pose.position.y = initial_y_ + 1.5;
        goal_pose.orientation.z = 0.707;
        goal_pose.orientation.w = 0.707;

        goal_msg.pose.pose = goal_pose;
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map";

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&NavigationClient::goal_result_callback, this, std::placeholders::_1);

        this->nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

    /**
     * @brief Callback for the result of the first navigation goal.
     * @param result The result of the goal execution.
     */
    void goal_result_callback(const GoalHandleNavigate::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            goal_reached_ = true;
        }
    }

    /**
     * @brief Callback for the odometry message, uses current odometry and compares to  goal to navigate the robot to
     * the construction zone.
     * @param msg The odometry message.
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_odom_ = msg->pose.pose.position.x;
        current_y_odom_ = msg->pose.pose.position.y;

        if (goal_reached_)
        {
            double target_x = 4.3;
            double target_y = -0.2;

            double angle_to_goal = atan2(target_y - current_y_odom_, target_x - current_x_odom_) + M_PI;

            tf2::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                                 msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
            tf2::Matrix3x3 mat(quat);
            double roll, pitch, yaw;
            mat.getRPY(roll, pitch, yaw);

            double yaw_error = normalize_angle(angle_to_goal - yaw);
            auto twist_msg = geometry_msgs::msg::Twist();

            if (!alignment_done_)
            {
                if (std::abs(yaw_error) > 0.05)
                {
                    twist_msg.angular.z = 0.3 * yaw_error / std::abs(yaw_error);
                    twist_msg.linear.x = 0.0;
                    cmd_vel_publisher_->publish(twist_msg);
                    RCLCPP_INFO(this->get_logger(), "\033[1;34mALIGNING TO GOAL...\033[0m");
                }
                else
                {
                    alignment_done_ = true;
                    RCLCPP_INFO(this->get_logger(), "\033[1;36mALIGNMENT COMPLETE, STARTING BACKWARD MOVEMENT...\033[0m");
                }
                return;
            }

            twist_msg.linear.x = -0.3;
            twist_msg.angular.z = (std::abs(yaw_error) > 0.05) ? 0.3 * yaw_error / std::abs(yaw_error) : 0.0;
            cmd_vel_publisher_->publish(twist_msg);

            double distance_to_goal = sqrt(pow(target_x - current_x_odom_, 2) + pow(target_y - current_y_odom_, 2));
            RCLCPP_INFO(this->get_logger(), "\033[1;35mDISTANCE TO GOAL: %.2f METERS\033[0m", distance_to_goal);

            if (distance_to_goal <= 0.3)
            {
                RCLCPP_INFO(this->get_logger(), "\033[1;32mCONSTRUCTION ZONE REACHED!\033[0m");
                twist_msg.linear.x = 0.0;
                twist_msg.angular.z = 0.0;
                cmd_vel_publisher_->publish(twist_msg);
                rclcpp::shutdown();
            }
        }
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

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<Localization>::SharedPtr localization_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool goal_reached_, localization_done_, alignment_done_;
    double current_x_odom_, current_y_odom_, initial_x_odom_, initial_y_odom_;
    double distance_traveled_, initial_x_, initial_y_;
};

/**
 * @brief Main function.
 * Initializes and runs the NavigationClient node.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavigationClient>());
    rclcpp::shutdown();
    return 0;
}