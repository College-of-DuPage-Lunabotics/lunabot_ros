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
        : Node("navigation_client"), start_localization_(true), start_navigation_(false), start_excavation_(false)
    {
        navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        localization_client_ = rclcpp_action::create_client<Localization>(this, "localization_action");
        excavation_client_ = rclcpp_action::create_client<Excavation>(this, "excavation_action");

        execution_timer_ =
            this->create_wall_timer(std::chrono::seconds(1), std::bind(&NavigationClient::execute, this));
    }

  private:

    /**
     * @brief Runs the main execution sequence.
     */
    void execute()
    {
        if (start_localization_)
        {
            request_localization();
        }
        else if (start_navigation_)
        {
            request_navigation();
        }
        else if (start_excavation_)
        {
            request_excavation();
        }
    }

    /**
     * @brief Sends localization request to localization server.
     */
    void request_localization()
    {
        if (!localization_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "\033[1;LOCALIZATION ACTION SERVER NOT AVAILABLE.\033[0m");
            return;
        }

        auto goal_msg = Localization::Goal();
        auto send_goal_options = rclcpp_action::Client<Localization>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&NavigationClient::handle_localization_result, this, std::placeholders::_1);

        localization_client_->async_send_goal(goal_msg, send_goal_options);
        start_localization_ = false;
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
            RCLCPP_INFO_ONCE(this->get_logger(), "\033[1;32mLOCALIZATION COMPLETE. INITIAL POSE: [%.2f, %.2f]\033[0m",
                             initial_x_, initial_y_);
            start_navigation_ = true;
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
        if (!navigation_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "\033[1;33mNAVIGATION ACTION SERVER NOT AVAILABLE.\033[0m");
            return;
        }

        std::this_thread::sleep_for(std::chrono::seconds(7)); // Wait for Nav2 to finish loading

        auto goal_msg = NavigateToPose::Goal();
        geometry_msgs::msg::Pose goal_pose;

        goal_pose.position.x = initial_x_ + 3.5;
        goal_pose.position.y = initial_y_ + 1.2;
        goal_pose.orientation.z = 0.707;
        goal_pose.orientation.w = 0.707;

        goal_msg.pose.pose = goal_pose;
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.header.frame_id = "map";

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&NavigationClient::handle_navigation_result, this, std::placeholders::_1);

        RCLCPP_INFO_ONCE(this->get_logger(), "\033[1;36mSENDING NAVIGATION GOAL TO EXCAVATION ZONE...\033[0m");
        navigation_client_->async_send_goal(goal_msg, send_goal_options);
        start_navigation_ = false;
    }

    /**
     * @brief Callback for the result of the navigation goal.
     * @param result The result of the goal execution.
     */
    void handle_navigation_result(const GoalHandleNavigate::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "\033[1;32mEXCAVATION ZONE REACHED. REQUESTING EXCAVATION...\033[0m");
            start_excavation_ = true;
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
        if (!excavation_client_->wait_for_action_server(std::chrono::seconds(1)))
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "\033[1;33mEXCAVATION ACTION SERVER NOT AVAILABLE.\033[0m");
            return;
        }

        auto goal_msg = Excavation::Goal();
        auto send_goal_options = rclcpp_action::Client<Excavation>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&NavigationClient::handle_excavation_result, this, std::placeholders::_1);

        excavation_client_->async_send_goal(goal_msg, send_goal_options);
        start_excavation_ = false;
    }

    /**
     * @brief Callback for the result of the excavation action.
     * @param result The result of the excavation action.
     */
    void handle_excavation_result(const GoalHandleExcavation::WrappedResult &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "\033[1;32mEXCAVATION SUCCESS!\033[0m");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "\033[1;31mEXCAVATION FAILED.\033[0m");
        }
    }

    rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
    rclcpp_action::Client<Localization>::SharedPtr localization_client_;
    rclcpp_action::Client<Excavation>::SharedPtr excavation_client_;
    rclcpp::TimerBase::SharedPtr execution_timer_;

    bool start_localization_, start_navigation_, start_excavation_;
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
