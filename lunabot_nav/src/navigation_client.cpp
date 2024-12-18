/**
 * @file navigation_client.cpp
 * @brief Sends navigation goals, retrieves initial pose from the localization server, and requests excavation action.
 */

#include <chrono>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "lunabot_msgs/action/excavation.hpp"
#include "lunabot_msgs/action/localization.hpp"

class NavigationClient : public rclcpp::Node
{
  public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using Localization = lunabot_msgs::action::Localization;
    using GoalHandleLocalization = rclcpp_action::ClientGoalHandle<Localization>;
    using Excavation = lunabot_msgs::action::Excavation;
    using GoalHandleExcavation = rclcpp_action::ClientGoalHandle<Excavation>;

    /**
     * @brief Constructor for the NavigationClient class.
     */
    NavigationClient()
        : Node("navigation_client"), goal_reached_(false), excavation_done_(false), localization_done_(false)
    {
        nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        localization_client_ = rclcpp_action::create_client<Localization>(this, "localization_action");
        excavation_client_ = rclcpp_action::create_client<Excavation>(this, "excavation_action");

        timer_ =
            this->create_wall_timer(std::chrono::seconds(1), std::bind(&NavigationClient::start_localization, this));
    }

  private:
    /**
     * @brief Initiates localization and retrieves the initial pose.
     */
    void start_localization()
    {
        if (!localization_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "\033[1;LOCALIZATION ACTION SERVER NOT AVAILABLE.\033[0m");
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
    void handle_localization_result(const GoalHandleLocalization::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            initial_x_ = result.result->x;
            initial_y_ = result.result->y;
            localization_done_ = true;
            RCLCPP_INFO_ONCE(this->get_logger(), "\033[1;32mLOCALIZATION COMPLETE. INITIAL POSE: [%.2f, %.2f]\033[0m",
                             initial_x_, initial_y_);
            send_navigation_goal();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "\033[1;31mLOCALIZATION FAILED.\033[0m");
            rclcpp::shutdown();
        }
    }

    /**
     * @brief Sends the navigation goal to the excavation zone.
     */
    void send_navigation_goal()
    {
        if (goal_reached_ || !nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "\033[1;33mNAVIGATION ACTION SERVER NOT AVAILABLE.\033[0m");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        geometry_msgs::msg::Pose goal_pose;

        goal_pose.position.x = initial_x_ + 3.7;
        goal_pose.position.y = initial_y_ + 2.0;
        goal_pose.orientation.z = 0.707;
        goal_pose.orientation.w = 0.707;

        goal_msg.pose.pose = goal_pose;
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map";

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&NavigationClient::goal_result_callback, this, std::placeholders::_1);

        RCLCPP_INFO_ONCE(this->get_logger(), "\033[1;36mSENDING NAVIGATION GOAL TO EXCAVATION ZONE...\033[0m");
        nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
    }

    /**
     * @brief Callback for the result of the navigation goal.
     * @param result The result of the goal execution.
     */
    void goal_result_callback(const GoalHandleNavigate::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "\033[1;32mEXCAVATION ZONE REACHED. REQUESTING EXCAVATION...\033[0m");
            goal_reached_ = true;

            request_excavation();
        }

        else if (result.code == rclcpp_action::ResultCode::ABORTED)
        {
            RCLCPP_ERROR(this->get_logger(), "\033[1;31mGOAL ABORTED, UNABLE TO REACH EXCAVATION ZONE\033[0m");
        }
    }

    /**
     * @brief Sends a request to the excavation server.
     */
    void request_excavation()
    {
        if (excavation_done_ || !excavation_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "\033[1;33mEXCAVATION ACTION SERVER NOT AVAILABLE.\033[0m");
            return;
        }

        auto goal_msg = Excavation::Goal();
        auto send_goal_options = rclcpp_action::Client<Excavation>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&NavigationClient::excavation_result_callback, this, std::placeholders::_1);

        excavation_client_->async_send_goal(goal_msg, send_goal_options);
    }

    /**
     * @brief Callback for the result of the excavation action.
     * @param result The result of the excavation action.
     */
    void excavation_result_callback(const GoalHandleExcavation::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "\033[1;32mEXCAVATION SUCCESS!\033[0m");
            excavation_done_ = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "\033[1;31mEXCAVATION FAILED.\033[0m");
        }
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<Localization>::SharedPtr localization_client_;
    rclcpp_action::Client<Excavation>::SharedPtr excavation_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool goal_reached_, excavation_done_, localization_done_;
    double initial_x_, initial_y_;
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
