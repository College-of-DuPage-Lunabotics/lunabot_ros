/**
 * @file livox_reorient.cpp
 * @author Grayson Arendt
 * @date 9/26/2026
 */

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "sensor_msgs/msg/imu.hpp"

#include <optional>
#include <string>
#include <vector>

/**
 * @class LivoxReorient
 * @brief Rotates Livox lidar and IMU data into the base_link orientation using the URDF mount
 * transform, so LIO sees an upright forward-facing sensor. Translation stays in LIO extrinsic_T.
 * Optionally drops lidar points inside a base_link box so the robot body never enters the map.
 */
class LivoxReorient : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for LivoxReorient.
   */
  LivoxReorient() : Node("livox_reorient"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");

    // Empty means use header.frame_id
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "");

    // Robot body box in base_link, [x, y, z] min and max, empty disables crop
    body_min_ = declare_parameter<std::vector<double>>("body_min", std::vector<double>{});
    body_max_ = declare_parameter<std::vector<double>>("body_max", std::vector<double>{});
    crop_body_ = body_min_.size() == 3 && body_max_.size() == 3;

    std::string input_lidar_topic =
      declare_parameter<std::string>("input_lidar_topic", "/livox/lidar");
    std::string input_imu_topic = declare_parameter<std::string>("input_imu_topic", "/livox/imu");
    std::string output_lidar_topic =
      declare_parameter<std::string>("output_lidar_topic", "/livox/lidar_body");
    std::string output_imu_topic =
      declare_parameter<std::string>("output_imu_topic", "/livox/imu_body");

    lidar_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input_lidar_topic, 20,
      std::bind(&LivoxReorient::lidar_callback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_imu_topic, 200, std::bind(&LivoxReorient::imu_callback, this, std::placeholders::_1));

    lidar_pub_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(output_lidar_topic, 20);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(output_imu_topic, 200);

    RCLCPP_INFO(
      get_logger(), "Rotating %s and %s into %s", input_lidar_topic.c_str(),
      input_imu_topic.c_str(), target_frame_.c_str());
  }

private:
  /**
   * @brief Looks up the rotation from source_frame into target_frame.
   * @param source_frame Frame the raw sensor data is in.
   * @return The rotation, or empty if TF does not have it yet.
   */
  std::optional<tf2::Matrix3x3> lookup_rotation(const std::string & source_frame)
  {
    if (source_frame.empty())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Message has an empty frame_id, set the lidar_frame or imu_frame parameter");
      return std::nullopt;
    }

    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform(target_frame_, source_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & e)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for transform %s -> %s: %s",
        source_frame.c_str(), target_frame_.c_str(), e.what());
      return std::nullopt;
    }

    const auto & q = transform.transform.rotation;
    tf2::Matrix3x3 rotation(tf2::Quaternion(q.x, q.y, q.z, q.w));

    double roll, pitch, yaw;
    rotation.getRPY(roll, pitch, yaw);
    const auto & t = transform.transform.translation;
    last_translation_ = tf2::Vector3(t.x, t.y, t.z);
    RCLCPP_INFO(
      get_logger(),
      "%s -> %s: rpy [%.3f, %.3f, %.3f], translation [%.3f, %.3f, %.3f] (set as LIO extrinsic_T)",
      source_frame.c_str(), target_frame_.c_str(), roll, pitch, yaw, t.x, t.y, t.z);

    return rotation;
  }

  /**
   * @brief Returns true if a point (base_link orientation, lidar origin) lies inside the body box.
   * @param p The rotated point.
   */
  bool inside_body(const tf2::Vector3 & p) const
  {
    const tf2::Vector3 b = p + lidar_offset_;  // shift to the base_link origin
    return b.x() >= body_min_[0] && b.x() <= body_max_[0] && b.y() >= body_min_[1] &&
           b.y() <= body_max_[1] && b.z() >= body_min_[2] && b.z() <= body_max_[2];
  }

  /**
   * @brief Callback for rotating lidar points and dropping robot body hits.
   * @param msg The received Livox point cloud.
   */
  void lidar_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
  {
    if (!lidar_rotation_)
    {
      lidar_rotation_ = lookup_rotation(lidar_frame_.empty() ? msg->header.frame_id : lidar_frame_);
      if (!lidar_rotation_)
      {
        return;
      }
      lidar_offset_ = last_translation_;
    }

    livox_ros_driver2::msg::CustomMsg out = *msg;
    out.header.frame_id = target_frame_;
    out.points.clear();
    out.points.reserve(msg->points.size());

    for (const auto & p : msg->points)
    {
      tf2::Vector3 rotated = *lidar_rotation_ * tf2::Vector3(p.x, p.y, p.z);
      if (crop_body_ && inside_body(rotated))
      {
        continue;
      }
      livox_ros_driver2::msg::CustomPoint q = p;
      q.x = rotated.x();
      q.y = rotated.y();
      q.z = rotated.z();
      out.points.push_back(q);
    }
    out.point_num = out.points.size();

    lidar_pub_->publish(out);
  }

  /**
   * @brief Callback for rotating IMU angular velocity and linear acceleration.
   * @param msg The received IMU message.
   */
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!imu_rotation_)
    {
      imu_rotation_ = lookup_rotation(imu_frame_.empty() ? msg->header.frame_id : imu_frame_);
      if (!imu_rotation_)
      {
        return;
      }
    }

    sensor_msgs::msg::Imu out = *msg;
    out.header.frame_id = target_frame_;

    tf2::Vector3 gyro =
      *imu_rotation_ *
      tf2::Vector3(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    out.angular_velocity.x = gyro.x();
    out.angular_velocity.y = gyro.y();
    out.angular_velocity.z = gyro.z();

    tf2::Vector3 accel = *imu_rotation_ * tf2::Vector3(
                                            msg->linear_acceleration.x, msg->linear_acceleration.y,
                                            msg->linear_acceleration.z);
    out.linear_acceleration.x = accel.x();
    out.linear_acceleration.y = accel.y();
    out.linear_acceleration.z = accel.z();

    imu_pub_->publish(out);
  }

  std::string target_frame_;
  std::string lidar_frame_;
  std::string imu_frame_;

  std::optional<tf2::Matrix3x3> lidar_rotation_;
  std::optional<tf2::Matrix3x3> imu_rotation_;
  tf2::Vector3 last_translation_{0.0, 0.0, 0.0};
  tf2::Vector3 lidar_offset_{0.0, 0.0, 0.0};

  bool crop_body_ = false;
  std::vector<double> body_min_;
  std::vector<double> body_max_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

/**
 * @brief Main function.
 * Initializes and spins the LivoxReorient node.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxReorient>());
  rclcpp::shutdown();
  return 0;
}
