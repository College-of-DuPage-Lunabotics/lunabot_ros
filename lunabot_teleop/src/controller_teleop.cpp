/**
 * @file controller_teleop.cpp
 * @author Grayson Arendt
 * @date 9/18/2024
 */

#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "SparkMax.hpp"

/**
 * @class ControllerTeleop
 * @brief Controls the robot physically via manual and autonomous commands using various controllers.
 */
class ControllerTeleop : public rclcpp::Node
{
  public:
    ControllerTeleop()
        : Node("motor_controller"), manual_enabled_(true), robot_disabled_(false), left_wheel_motor_("can0", 1),
          right_wheel_motor_("can0", 2), lift_actuator_motor_("can0", 3), tilt_actuator_left_motor_("can0", 4),
          tilt_actuator_right_motor_("can0", 5)
    {
        velocity_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&ControllerTeleop::velocity_callback, this, std::placeholders::_1));
        joystick_subscriber_ = create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&ControllerTeleop::joy_callback, this, std::placeholders::_1));

        declare_and_get_parameters();
        apply_controller_mode();

        RCLCPP_INFO(get_logger(), "\033[1;35mMANUAL CONTROL:\033[0m \033[1;32mENABLED\033[0m");
    }

  private:
    /**
     * @brief Declares and retrieves controller mode parameters.
     */
    void declare_and_get_parameters()
    {
        declare_parameter("xbox_mode", true);
        declare_parameter("ps4_mode", false);
        declare_parameter("switch_mode", false);
        declare_parameter("outdoor_mode", false);

        get_parameter("xbox_mode", xbox_mode_);
        get_parameter("ps4_mode", ps4_mode_);
        get_parameter("switch_mode", switch_mode_);
        get_parameter("outdoor_mode", outdoor_mode_);
    }

    /**
     * @brief Applies the correct controller mode based on parameters.
     */
    void apply_controller_mode()
    {
        if (xbox_mode_)
            RCLCPP_INFO(get_logger(), "XBOX MODE");
        else if (ps4_mode_)
            RCLCPP_INFO(get_logger(), "PS4 MODE");
        else if (switch_mode_)
            RCLCPP_INFO(get_logger(), "SWITCH MODE");
        else
            RCLCPP_ERROR(get_logger(), "\033[1;31mNO CONTROLLER SELECTED, CHECK LAUNCH PARAMETERS\033[0m");
    }

    /**
     * @brief Controls the robot based on joystick input.
     */
    void control_robot()
    {
        if (home_button_)
            robot_disabled_ = true;

        lift_actuator_speed_ = (d_pad_vertical_ == 1.0) ? -0.3 : (d_pad_vertical_ == -1.0) ? 0.3 : 0.0;
        tilt_actuator_speed_ = (right_joystick_y_ > 0.1) ? -0.3 : (right_joystick_y_ < -0.1) ? 0.3 : 0.0;

        speed_multiplier_ = (x_button_ && y_button_) ? 0.3 : (x_button_ ? 0.1 : 0.6);

        if (switch_mode_)
        {
            left_speed_ = left_joystick_y_ - left_joystick_x_;
            right_speed_ = left_joystick_y_ + left_joystick_x_;
        }
        else
        {
            right_trigger_ = (1.0 - right_trigger_) / 2.0;
            left_trigger_ = (1.0 - left_trigger_) / 2.0;

            if (right_trigger_ != 0.0)
            {
                left_speed_ = right_trigger_ - left_joystick_x_;
                right_speed_ = right_trigger_ + left_joystick_x_;
            }
            else if (left_trigger_ != 0.0)
            {
                left_speed_ = -(left_trigger_ - left_joystick_x_);
                right_speed_ = -(left_trigger_ + left_joystick_x_);
            }
            else
            {
                left_speed_ = -left_joystick_x_;
                right_speed_ = left_joystick_x_;
            }
        }

        left_wheel_motor_.SetDutyCycle(std::clamp(left_speed_ * speed_multiplier_, -1.0, 1.0));
        right_wheel_motor_.SetDutyCycle(std::clamp(right_speed_ * speed_multiplier_, -1.0, 1.0));
        lift_actuator_motor_.SetDutyCycle(std::clamp(lift_actuator_speed_, -1.0, 1.0));
        tilt_actuator_left_motor_.SetDutyCycle(std::clamp(tilt_actuator_speed_, -1.0, 1.0));
        tilt_actuator_right_motor_.SetDutyCycle(std::clamp(tilt_actuator_speed_, -1.0, 1.0));
    }

    /**
     * @brief Joystick input processing.
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
        auto clock = rclcpp::Clock();

        share_button_ = get_button(joy_msg, {9, 8, 9});
        menu_button_ = get_button(joy_msg, {10, 9, 10});
        home_button_ = get_button(joy_msg, {11, 10, 8});
        a_button_ = get_button(joy_msg, {1, 0, 0});
        b_button_ = get_button(joy_msg, {0, 2, 1});
        x_button_ = get_button(joy_msg, {2, 3, 2});
        y_button_ = get_button(joy_msg, {3, 2, 3});
        left_bumper_ = get_button(joy_msg, {4, 4, 4});
        right_bumper_ = get_button(joy_msg, {5, 5, 5});

        d_pad_horizontal_ = get_axis(joy_msg, {4, 6, 6});
        d_pad_vertical_ = get_axis(joy_msg, {5, 7, 7});

        if (share_button_)
        {
            manual_enabled_ = true;
            RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "\033[1;35mMANUAL CONTROL:\033[0m \033[1;32mENABLED\033[0m");
        }

        if (menu_button_)
        {
            manual_enabled_ = false;
            RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "\033[1;33mAUTONOMOUS CONTROL:\033[0m \033[1;32mENABLED\033[0m");
        }

        if (manual_enabled_)
        {
            left_joystick_x_ = joy_msg->axes[0];
            left_joystick_y_ = joy_msg->axes[1];
            right_joystick_y_ = joy_msg->axes[4];
            left_trigger_ = joy_msg->axes[2];
            right_trigger_ = joy_msg->axes[5];

            if (robot_disabled_)
            {
                RCLCPP_ERROR(get_logger(), "\033[1;31mROBOT DISABLED\033[0m");
            }
            else
            {
                SparkMax::Heartbeat();
            }

            control_robot();
        }
    }

    /**
     * @brief Velocity message callback for navigation control.
     */
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr velocity_msg)
    {
        if (!manual_enabled_)
        {
            SparkMax::Heartbeat();
            double linear_velocity = velocity_msg->linear.x;
            double angular_velocity = velocity_msg->angular.z;
            double wheel_radius = outdoor_mode_ ? 0.2 : 0.127;
            double wheel_distance = 0.5;

            double velocity_left_cmd =
                -0.05 * (linear_velocity + angular_velocity * wheel_distance / 2.0) / wheel_radius;
            double velocity_right_cmd =
                -0.05 * (linear_velocity - angular_velocity * wheel_distance / 2.0) / wheel_radius;

            left_wheel_motor_.SetDutyCycle(std::clamp(velocity_left_cmd, -1.0, 1.0));
            right_wheel_motor_.SetDutyCycle(std::clamp(velocity_right_cmd, -1.0, 1.0));
        }
    }

    /**
     * @brief Helper to retrieve button value based on controller mode.
     */
    int get_button(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const std::initializer_list<int> &mappings)
    {
        size_t index = xbox_mode_ ? 2 : ps4_mode_ ? 1 : 0;
        return joy_msg->buttons[*(mappings.begin() + index)];
    }

    /**
     * @brief Helper to retrieve axis value based on controller mode.
     */
    double get_axis(const sensor_msgs::msg::Joy::SharedPtr &joy_msg, const std::initializer_list<int> &mappings)
    {
        size_t index = xbox_mode_ ? 2 : ps4_mode_ ? 1 : 0;
        return joy_msg->axes[*(mappings.begin() + index)];
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;

    bool manual_enabled_, robot_disabled_;
    bool xbox_mode_, ps4_mode_, switch_mode_, outdoor_mode_;
    double speed_multiplier_, left_speed_, right_speed_, lift_actuator_speed_, tilt_actuator_speed_;
    bool home_button_, share_button_, menu_button_, a_button_, b_button_, x_button_, y_button_;
    double left_trigger_, right_trigger_, left_bumper_, right_bumper_, d_pad_vertical_, d_pad_horizontal_;
    double left_joystick_x_, left_joystick_y_, right_joystick_y_;

    SparkMax left_wheel_motor_;
    SparkMax right_wheel_motor_;
    SparkMax lift_actuator_motor_;
    SparkMax tilt_actuator_left_motor_;
    SparkMax tilt_actuator_right_motor_;
};

/**
 * @brief Main function
 * Initializes and spins the ControllerTeleop node.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}