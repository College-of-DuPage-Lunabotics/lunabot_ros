/**
 * @file localization_server.cpp
 * @author Grayson Arendt
 * @date 12/19/2024
 */

#include <chrono>
#include <cmath>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "lunabot_msgs/action/localization.hpp"

/**
 * @class LocalizationServer
 * @brief Provides localization using AprilTags and manages an action server for localization feedback.
 */
class LocalizationServer : public rclcpp::Node
{
  public:
    using Localization = lunabot_msgs::action::Localization;
    using GoalHandleLocalization = rclcpp_action::ServerGoalHandle<Localization>;

    /**
     * @brief Constructor for the LocalizationServer class.
     */
    LocalizationServer()
        : Node("localization_server"), d455_tag_7_detected_(false), d455_tag_11_detected_(false),
          d456_tag_7_detected_(false), d456_tag_11_detected_(false), turn_direction_set_(false), turn_clockwise_(false),
          timer_started_(false), success_(false)
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

        localization_timer_ =
            create_wall_timer(std::chrono::milliseconds(100), std::bind(&LocalizationServer::localize, this));
    }

  private:
    /**
     * @brief Handles goal requests from clients.
     * @return Goal response (accept or reject).
     */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Localization::Goal>)
    {
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

        while (!success_ && rclcpp::ok())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (success_)
        {
            result->x = depth_distance_;
            result->y = -lateral_distance_ + 1.0;
            result->success = true;
            goal_handle->succeed(result);
        }
    }

    /**
     * @brief Aligns the robot to face east with the D455 camera facing AprilTag with tag 7.
     */
    void localize()
    {
        if (!timer_started_)
        {
            start_time_ = now();
            timer_started_ = true;
        }

        if (!turn_direction_set_)
        {
            success_ = d455_tag_7_detected_;
            turn_clockwise_ = !d456_tag_11_detected_;
            turn_direction_set_ = true;
        }

        twist.angular.z = turn_clockwise_ ? 0.3 : -0.3;

        if (d455_tag_7_detected_ && std::abs(normalize_angle(tag_7_yaw_)) < 0.025)
        {
            success_ = true;
            twist.angular.z = 0.0;
        }

        if ((now() - start_time_).seconds() < 30.0)
        {
            cmd_vel_publisher_->publish(twist);
        }

        else
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "\033[1;33mLOCALIZATION TIMED OUT.\033[0m");
        }
    }

    /**
     * @brief Detects AprilTags in images from the D455 camera.
     * @param input_image Image message from the D455 camera.
     */
    void d455_detect_apriltag(const sensor_msgs::msg::Image::SharedPtr input_image)
    {
        process_apriltag(input_image, d455_tag_7_detected_, d455_tag_11_detected_, d455_overlay_publisher_, true);
    }

    /**
     * @brief Detects AprilTags in images from the D456 camera.
     * @param input_image Image message from the D456 camera.
     */
    void d456_detect_apriltag(const sensor_msgs::msg::Image::SharedPtr input_image)
    {
        process_apriltag(input_image, d456_tag_7_detected_, d456_tag_11_detected_, d456_overlay_publisher_, false);
    }

    /**
     * @brief Processes detected AprilTags, calculates pose, and publishes overlay image.
     * @param input_image Input image message.
     * @param tag_7_detected_ Flag for tag 7 detection.
     * @param tag_11_detected_ Flag for tag 11 detection
     * @param overlay_publisher Publisher for overlay image.
     * @param calculate_tag_ Flag to determine whether to calculate yaw and distances for the detected tag.
     */
    void process_apriltag(const sensor_msgs::msg::Image::SharedPtr &input_image, bool &tag_7_detected_,
                          bool &tag_11_detected_,
                          const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &overlay_publisher,
                          bool calculate_tag_)
    {
        try
        {
            auto currentImage_ptr = cv_bridge::toCvCopy(input_image, input_image->encoding);
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
                    tag_7_detected_ = (tagId == 7 ? true : tag_7_detected_);
                    tag_11_detected_ = (tagId == 11 ? true : tag_11_detected_);

                    if (calculate_tag_ && tag_7_detected_)
                    {
                        calculate_distances(tvecs[i], lateral_distance_, depth_distance_);
                        calculate_yaw(rvecs[i], tag_7_yaw_);
                        tag_7_yaw_ = normalize_angle(tag_7_yaw_);
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
                tag_7_detected_ = false;
                tag_11_detected_ = false;
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
     * @param tag_7_yaw_ Calculated yaw for tag_7.
     */
    void calculate_yaw(const cv::Vec3d &rvec, double &tag_7_yaw_)
    {
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        tag_7_yaw_ = asin(rotation_matrix.at<double>(2, 0));
    }

    bool d455_tag_7_detected_, d455_tag_11_detected_, d456_tag_7_detected_, d456_tag_11_detected_;
    bool turn_direction_set_, turn_clockwise_, timer_started_, success_;
    double lateral_distance_, depth_distance_, tag_7_yaw_;

    rclcpp::Time start_time_;
    rclcpp::TimerBase::SharedPtr localization_timer_;
    geometry_msgs::msg::Twist twist;
    rclcpp_action::Server<Localization>::SharedPtr action_server_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr d455_image_subscriber_, d456_image_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr d455_overlay_publisher_, d456_overlay_publisher_;
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
