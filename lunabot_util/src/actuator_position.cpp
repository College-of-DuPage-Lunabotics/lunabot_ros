/**
 * @file actuator_position.cpp
 * @author Grayson Arendt
 * @date 5/16/2025
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <chrono>

class ActuatorPosition : public rclcpp::Node
{
public:
  ActuatorPosition()
  : Node("actuator_position_node")
  {
    position_pub_ = create_publisher<std_msgs::msg::Float64>("/encoder_pos", 10);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/teensy/imu/data", 10);

    declare_parameter("can_interface", "can0");
    get_parameter("can_interface", can_iface_);

    if (!open_can()) {
      RCLCPP_FATAL(get_logger(), "Failed to open %s", can_iface_.c_str());
      rclcpp::shutdown();
      return;
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1), std::bind(&ActuatorPosition::poll_can, this));
  }

  ~ActuatorPosition() override
  {
    if (can_sock_ >= 0) {::close(can_sock_);}
  }

private:
  bool open_can()
  {
    can_sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_sock_ < 0) {return false;}

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, can_iface_.c_str(), IFNAMSIZ - 1);
    if (::ioctl(can_sock_, SIOCGIFINDEX, &ifr) < 0) {return false;}

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(can_sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      return false;
    }

    can_filter flt[2];
    flt[0] = {0x200, CAN_SFF_MASK}; // encoder
    flt[1] = {0x201, CAN_SFF_MASK}; // IMU quaternion
    ::setsockopt(can_sock_, SOL_CAN_RAW, CAN_RAW_FILTER, &flt, sizeof(flt));
    return true;
  }

  void poll_can()
  {
    can_frame f{};
    if (::read(can_sock_, &f, sizeof(f)) != sizeof(f)) {return;}

    if (f.can_id == 0x200 && f.can_dlc == 4) {
      handle_encoder(f);
    } else if (f.can_id == 0x201 && f.can_dlc == 8) {handle_quat(f);}
  }

  void handle_encoder(const can_frame & f)
  {
    int32_t pos_um;
    std::memcpy(&pos_um, f.data, 4);

    if (!offset_set_) {offset_ = pos_um; offset_set_ = true;}
    pos_um -= offset_;

    std_msgs::msg::Float64 m;
    m.data = static_cast<double>(pos_um) / 1000.0;
    position_pub_->publish(m);
  }

  void handle_quat(const can_frame &f)
  {
    int16_t q16[4]; // w, x, y, z from Teensy
    std::memcpy(q16, f.data, 8);
  
    /* scale to double */
    double w_s = q16[0] / 10000.0;
    double x_s = q16[1] / 10000.0;
    double y_s = q16[2] / 10000.0;
    double z_s = q16[3] / 10000.0;

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = now();
    imu_msg.header.frame_id = "imu_link";
  
    imu_msg.orientation.w = w_s;
    imu_msg.orientation.x = x_s;
    imu_msg.orientation.y = y_s;
    imu_msg.orientation.z = z_s;
  
    const double var = 0.02 * 0.02; // ≈ 1 deg²
    imu_msg.orientation_covariance[0] =
    imu_msg.orientation_covariance[4] =
    imu_msg.orientation_covariance[8] = var;
  
    imu_msg.angular_velocity_covariance[0]    = -1.0;
    imu_msg.linear_acceleration_covariance[0] = -1.0;
  
    imu_pub_->publish(imu_msg);
  }  

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string can_iface_;
  int can_sock_ = -1;

  int32_t offset_ = 0;
  bool offset_set_ = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActuatorPosition>());
  rclcpp::shutdown();
  return 0;
}
