/**
 * @file lio_odom_to_base.cpp
 * @author Grayson Arendt
 * @date 10/28/2026
 */

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <optional>
#include <string>
#include <vector>

/**
 * @class LioOdomToBase
 * @brief Converts LIO odometry of the IMU (inside the Mid-360) into odometry of base_link.
 * LIO estimates the pose of the IMU, so this node shifts it by the fixed IMU lever arm and
 * publishes odom -> base_link. It also publishes static frames for the LIO odom and body frames
 * so LIO point clouds stay viewable.
 */
class LioOdomToBase : public rclcpp::Node
{
  static constexpr double twist_filter_ = 0.3;

public:
  /**
   * @brief Constructor for LioOdomToBase.
   */
  LioOdomToBase()
  : Node("lio_odom_to_base"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(this),
    static_broadcaster_(this)
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "livox_frame");
    lio_odom_frame_ = declare_parameter<std::string>("lio_odom_frame", "lio_init");
    lio_body_frame_ = declare_parameter<std::string>("lio_body_frame", "lio_imu");

    // IMU position in the lidar frame, Mid-360 datasheet value
    std::vector<double> imu_offset = declare_parameter<std::vector<double>>(
      "imu_offset", std::vector<double>{0.011, 0.02329, -0.04412});
    if (imu_offset.size() != 3)
    {
      RCLCPP_WARN(get_logger(), "imu_offset must have 3 values, using zero offset");
      imu_offset = {0.0, 0.0, 0.0};
    }
    imu_offset_ = tf2::Vector3(imu_offset[0], imu_offset[1], imu_offset[2]);

    std::string input_odom_topic =
      declare_parameter<std::string>("input_odom_topic", "/lio_odom_imu");
    std::string output_odom_topic =
      declare_parameter<std::string>("output_odom_topic", "/lio_odom");

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_odom_topic, 50,
      std::bind(&LioOdomToBase::odom_callback, this, std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_odom_topic, 50);

    RCLCPP_INFO(
      get_logger(), "Converting %s (IMU pose) into %s (%s pose)", input_odom_topic.c_str(),
      output_odom_topic.c_str(), base_frame_.c_str());
  }

private:
  /**
   * @brief Looks up the IMU position in base_link and publishes the static LIO frames.
   * @return True once the lever arm is known.
   */
  bool init_lever_arm()
  {
    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform(base_frame_, lidar_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for transform %s -> %s: %s",
        lidar_frame_.c_str(), base_frame_.c_str(), e.what());
      return false;
    }

    const auto & q = transform.transform.rotation;
    const auto & t = transform.transform.translation;
    tf2::Matrix3x3 rotation(tf2::Quaternion(q.x, q.y, q.z, q.w));
    lever_arm_ = tf2::Vector3(t.x, t.y, t.z) + rotation * imu_offset_;

    RCLCPP_INFO(
      get_logger(), "IMU position in %s: [%.4f, %.4f, %.4f]", base_frame_.c_str(),
      lever_arm_->x(), lever_arm_->y(), lever_arm_->z());

    // LIO frames have the base_link orientation, offset by the lever arm
    std::vector<geometry_msgs::msg::TransformStamped> static_transforms(2);
    static_transforms[0].header.frame_id = odom_frame_;
    static_transforms[0].child_frame_id = lio_odom_frame_;
    static_transforms[1].header.frame_id = base_frame_;
    static_transforms[1].child_frame_id = lio_body_frame_;
    for (auto & st : static_transforms)
    {
      st.header.stamp = now();
      st.transform.translation.x = lever_arm_->x();
      st.transform.translation.y = lever_arm_->y();
      st.transform.translation.z = lever_arm_->z();
      st.transform.rotation.w = 1.0;
    }
    static_broadcaster_.sendTransform(static_transforms);

    return true;
  }

  /**
   * @brief Fills the base_link twist from consecutive poses, low pass filtered (LIO leaves it zero).
   * @param out Odometry message to fill.
   * @param position Base position in odom.
   * @param rotation Base orientation in odom.
   */
  void fill_twist(nav_msgs::msg::Odometry & out, const tf2::Vector3 & position,
    const tf2::Quaternion & rotation)
  {
    const rclcpp::Time stamp(out.header.stamp);
    const double yaw = tf2::getYaw(rotation);

    if (last_stamp_)
    {
      const double dt = (stamp - *last_stamp_).seconds();
      if (dt > 1e-3 && dt < 0.5)
      {
            const tf2::Vector3 delta = position - last_position_;
        const double forward = delta.x() * std::cos(yaw) + delta.y() * std::sin(yaw);
        const double turn = std::atan2(std::sin(yaw - last_yaw_), std::cos(yaw - last_yaw_));
        linear_x_ += twist_filter_ * (forward / dt - linear_x_);
        angular_z_ += twist_filter_ * (turn / dt - angular_z_);
      }
    }
    last_stamp_ = stamp;
    last_position_ = position;
    last_yaw_ = yaw;

    out.twist.twist.linear.x = linear_x_;
    out.twist.twist.angular.z = angular_z_;
  }

  /**
   * @brief Callback for shifting the LIO IMU pose to base_link.
   * @param msg The received LIO odometry.
   */
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!lever_arm_ && !init_lever_arm())
    {
      return;
    }

    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;
    tf2::Quaternion rotation(q.x, q.y, q.z, q.w);

    tf2::Vector3 base_position =
      *lever_arm_ + tf2::Vector3(p.x, p.y, p.z) - tf2::quatRotate(rotation, *lever_arm_);

    nav_msgs::msg::Odometry out = *msg;
    out.header.frame_id = odom_frame_;
    out.child_frame_id = base_frame_;
    out.pose.pose.position.x = base_position.x();
    out.pose.pose.position.y = base_position.y();
    out.pose.pose.position.z = base_position.z();
    fill_twist(out, base_position, rotation);
    odom_pub_->publish(out);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = out.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = base_position.x();
    transform.transform.translation.y = base_position.y();
    transform.transform.translation.z = base_position.z();
    transform.transform.rotation = q;
    tf_broadcaster_.sendTransform(transform);
  }

  std::string base_frame_;
  std::string odom_frame_;
  std::string lidar_frame_;
  std::string lio_odom_frame_;
  std::string lio_body_frame_;

  tf2::Vector3 imu_offset_;
  std::optional<tf2::Vector3> lever_arm_;

  std::optional<rclcpp::Time> last_stamp_;
  tf2::Vector3 last_position_;
  double last_yaw_ = 0.0;
  double linear_x_ = 0.0;
  double angular_z_ = 0.0;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

/**
 * @brief Main function.
 * Initializes and spins the LioOdomToBase node.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LioOdomToBase>());
  rclcpp::shutdown();
  return 0;
}
