/**
 * @file localization_server.cpp
 * @author Grayson Arendt
 * @date 11/11/2024
 */

#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "lunabot_system/action/localization.hpp"

/**
 * @class LocalizationServer
 * @brief Provides localization using AprilTags and manages an action server for localization feedback.
 */
class LocalizationServer : public rclcpp::Node
{
  public:
    using Localization = lunabot_system::action::Localization;
    using GoalHandleLocalization = rclcpp_action::ServerGoalHandle<Localization>;

    /**
     * @brief Constructor for the LocalizationServer class.
     */
    LocalizationServer()
        : Node("localization_server"), d455_tag1_detected_(false), d455_tag2_detected_(false),
          d456_tag1_detected_(false), d456_tag2_detected_(false), turn_direction_set_(false), aligned_(false),
          alignment_started_(false), goal_received_(false), initial_rotation_started_(false)
    {
        d455_image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
            "d455/color/image_raw", 10,
            std::bind(&LocalizationServer::d455_detect_apriltag, this, std::placeholders::_1));
        d456_image_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
            "d456/color/image_raw", 10,
            std::bind(&LocalizationServer::d456_detect_apriltag, this, std::placeholders::_1));

        d455_overlay_publisher_ = create_publisher<sensor_msgs::msg::Image>("d455/apriltag/overlay_image", 10);
        d456_overlay_publisher_ = create_publisher<sensor_msgs::msg::Image>("d456/apriltag/overlay_image", 10);
        cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        action_server_ = rclcpp_action::create_server<Localization>(
            this, "localization_action",
            std::bind(&LocalizationServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&LocalizationServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&LocalizationServer::handle_accepted, this, std::placeholders::_1));

        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&LocalizationServer::align_robot, this));
    }

  private:
    /**
     * @brief Handles goal requests from clients.
     * @return Goal response (accept or reject).
     */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Localization::Goal>)
    {
        goal_received_ = true;
        alignment_started_ = false;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /**
     * @brief Handles cancellation requests from clients.
     * @return Cancel response (accept or reject).
     */
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleLocalization>)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /**
     * @brief Accepts and starts the goal execution.
     * @param goal_handle Handle for the current goal.
     */
    void handle_accepted(const std::shared_ptr<GoalHandleLocalization> goal_handle)
    {
        std::thread{std::bind(&LocalizationServer::execute, this, goal_handle)}.detach();
    }

    /**
     * @brief Executes the localization process.
     * @param goal_handle Handle for the current goal.
     */
    void execute(const std::shared_ptr<GoalHandleLocalization> goal_handle)
    {
        auto result = std::make_shared<Localization::Result>();

        while (!aligned_ && rclcpp::ok())
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (aligned_)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "\033[1;32mLOCALIZATION SUCCESS!\033[0m");
            result->success = true;
            result->x = -lateral_distance_;
            result->y = depth_distance_;
            result->yaw = tag1_yaw;
            goal_handle->succeed(result);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "\033[1;31mLOCALIZATION FAILED!\033[0m");
            result->success = false;
            goal_handle->abort(result);
        }
    }
    /**
     * @brief Aligns the robot with the detected AprilTag.
     */

    void align_robot()
    {
        geometry_msgs::msg::Twist twist;

        if (!alignment_started_)
        {
            alignment_start_time_ = now();
            alignment_started_ = true;
        }

        double elapsed_time = (now() - alignment_start_time_).seconds();

        if (elapsed_time > 60.0 && !aligned_)
        {
            twist.angular.z = 0.0;
            cmd_vel_publisher_->publish(twist);
            aligned_ = false;
            return;
        }

        if (aligned_)
        {
            twist.angular.z = 0.0;
            cmd_vel_publisher_->publish(twist);
            return;
        }

        if (d455_tag1_detected_)
        {
            double yaw_error = normalize_angle(tag1_yaw);
            if (std::abs(yaw_error) > 0.025)
            {
                twist.angular.z = yaw_error > 0 ? -0.1 : 0.1;
            }
            else
            {
                twist.angular.z = 0.0;
                aligned_ = true;
                RCLCPP_INFO(this->get_logger(), "\033[1;34mALIGNMENT COMPLETE WITH TAG 7.\033[0m");
            }
        }
        else if (!turn_direction_set_)
        {
            if (!d455_tag1_detected_ && !d456_tag1_detected_ && !initial_rotation_started_)
            {
                initial_rotation_started_ = true;
                RCLCPP_INFO(this->get_logger(), "\033[1;33mNO TAGS DETECTED. STARTING SLOW ROTATION...\033[0m");

                twist.angular.z = -0.01;
                cmd_vel_publisher_->publish(twist);
                return;
            }

            turn_counterclockwise_ = d455_tag1_detected_ ? false : (d456_tag1_detected_ || d456_tag2_detected_);
            turn_direction_set_ = true;
        }
        else
        {
            twist.angular.z = turn_counterclockwise_ ? 0.1 : -0.1;
        }

        cmd_vel_publisher_->publish(twist);
    }
    /**
     * @brief Detects AprilTags in images from the D455 camera.
     * @param inputImage Image message from the D455 camera.
     */

    void d455_detect_apriltag(const sensor_msgs::msg::Image::SharedPtr inputImage)
    {
        process_apriltag(inputImage, d455_tag1_detected_, d455_tag2_detected_, d455_overlay_publisher_, true);
    }

    /**
     * @brief Detects AprilTags in images from the D456 camera.
     * @param inputImage Image message from the D456 camera.
     */
    void d456_detect_apriltag(const sensor_msgs::msg::Image::SharedPtr inputImage)
    {
        process_apriltag(inputImage, d456_tag1_detected_, d456_tag2_detected_, d456_overlay_publisher_, false);
    }

    /**
     * @brief Processes detected AprilTags, calculates pose, and publishes overlay image.
     * @param inputImage Image message.
     * @param tag1_detected Flag for detecting tag1.
     * @param tag2_detected Flag for detecting tag2.
     * @param overlay_publisher Publisher for overlay image.
     * @param calculate_yaw_ Boolean to calculate yaw for detected tags.
     */
    void process_apriltag(const sensor_msgs::msg::Image::SharedPtr &inputImage, bool &tag1_detected,
                          bool &tag2_detected,
                          const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &overlay_publisher,
                          bool calculate_yaw_)
    {
        try
        {
            auto currentImage_ptr = cv_bridge::toCvCopy(inputImage, inputImage->encoding);
            auto outputImage = currentImage_ptr->image.clone();
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners;

            cv::Mat cameraMatrix = (cv::Mat1d(3, 3) << 383.4185742519996, 0, 309.4326377845713, 0, 385.0909007102088,
                                    240.749949733094, 0, 0, 1);
            cv::Mat distortionCoefficients = (cv::Mat1d(1, 5) << -0.06792929080519726, 0.08058277259698843,
                                              -0.001690544521662593, -0.0008235437909836152, -0.04417756393089296);
            std::vector<cv::Vec3d> rvecs, tvecs;

            auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
            cv::aruco::detectMarkers(currentImage_ptr->image, dictionary, markerCorners, markerIds);
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.235, cameraMatrix, distortionCoefficients, rvecs,
                                                 tvecs);

            if (!markerIds.empty())
            {
                for (size_t i = 0; i < markerIds.size(); ++i)
                {
                    int tagId = markerIds[i];
                    tag1_detected = (tagId == 7 ? true : tag1_detected);
                    tag2_detected = (tagId == 11 ? true : tag2_detected);

                    if (calculate_yaw_ && tag1_detected)
                    {
                        calculate_distances(tvecs[i], lateral_distance_, depth_distance_);
                        calculate_yaw(rvecs[i], tag1_yaw);
                        tag1_yaw = normalize_angle(tag1_yaw);
                    }
                }
                cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
                for (size_t i = 0; i < markerIds.size(); ++i)
                {
                    cv::aruco::drawAxis(outputImage, cameraMatrix, distortionCoefficients, rvecs[i], tvecs[i], 0.1);
                }
                overlay_publisher->publish(
                    *cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", outputImage).toImageMsg());
            }
            else
            {
                tag1_detected = false;
                tag2_detected = false;
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "\033[1;31mERROR PROCESSING IMAGE: %s\033[0m", e.what());
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

    /**
     * @brief Calculates lateral and depth distances from translation vector.
     * @param tvec Translation vector.
     * @param lateral_distance Calculated lateral distance.
     * @param depth_distance Calculated depth distance.
     */
    void calculate_distances(const cv::Vec3d &tvec, double &lateral_distance, double &depth_distance)
    {
        lateral_distance = tvec[0];
        depth_distance = tvec[2];
    }

    /**
     * @brief Calculates yaw from rotation vector.
     * @param rvec Rotation vector.
     * @param tag1_yaw Calculated yaw for tag1.
     */
    void calculate_yaw(const cv::Vec3d &rvec, double &tag1_yaw)
    {
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        tag1_yaw = asin(rotation_matrix.at<double>(2, 0));
    }

    bool d455_tag1_detected_, d455_tag2_detected_, d456_tag1_detected_, d456_tag2_detected_;
    bool turn_direction_set_, turn_counterclockwise_, aligned_, goal_received_, alignment_started_,
        initial_rotation_started_;
    double lateral_distance_, depth_distance_, tag1_yaw;

    rclcpp::Time alignment_start_time_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Server<Localization>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr d455_image_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr d456_image_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr d455_overlay_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr d456_overlay_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

/**
 * @brief Main function.
 * Initializes and runs the LocalizationServer node.
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationServer>());
    rclcpp::shutdown();
    return 0;
}
