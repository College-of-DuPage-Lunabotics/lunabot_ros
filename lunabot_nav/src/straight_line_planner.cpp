/**
 * @file straight_line_planner.cpp
 * @author Grayson Arendt
 * @brief Based off of tutorial: https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 * @date 02/15/2025
 */

#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace straight_line_planner
{

/**
 * @class StraightLine
 * @brief A global planner plugin for generating straight line paths.
 */
class StraightLine : public nav2_core::GlobalPlanner
{
public:
  /**
   * @brief Configures the planner with necessary parameters.
   * @param parent The parent node.
   * @param name The name of the planner.
   * @param tf The transform buffer.
   * @param costmap_ros The costmap.
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    global_frame_ = costmap_ros->getGlobalFrameID();

    node_->declare_parameter(name_ + ".resolution", 0.1);
    node_->get_parameter(name_ + ".resolution", resolution_);
  }

  /**
   * @brief Cleans up the planner resources when deactivated.
   */
  void cleanup()
  {
    RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin %s", name_.c_str());
  }

  /**
   * @brief Activates the planner.
   */
  void activate()
  {
    RCLCPP_INFO(node_->get_logger(), "Activating plugin %s", name_.c_str());
  }

  /**
   * @brief Deactivates the planner.
   */
  void deactivate()
  {
    RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s", name_.c_str());
  }

  /**
   * @brief Creates a straight-line path from the start to the goal pose.
   * @param start The start pose.
   * @param goal The goal pose.
   * @return The generated path as a sequence of poses.
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal)
  {
    nav_msgs::msg::Path global_path;

    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    int total_points =
      std::hypot(
      goal.pose.position.x - start.pose.position.x,
      goal.pose.position.y - start.pose.position.y) /
      resolution_;

    double x_increment = (goal.pose.position.x - start.pose.position.x) / total_points;
    double y_increment = (goal.pose.position.y - start.pose.position.y) / total_points;

    for (int i = 0; i < total_points; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = start.pose.position.x + x_increment * i;
      pose.pose.position.y = start.pose.position.y + y_increment * i;
      pose.pose.position.z = 0.0;

      double yaw = std::atan2(y_increment, x_increment);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();

      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      global_path.poses.push_back(pose);
    }

    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header.stamp = node_->now();
    goal_pose.header.frame_id = global_frame_;
    global_path.poses.push_back(goal_pose);

    return global_path;
  }

private:
  nav2_util::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string global_frame_, name_;

  double resolution_;
};
} // namespace straight_line_planner

PLUGINLIB_EXPORT_CLASS(straight_line_planner::StraightLine, nav2_core::GlobalPlanner)
