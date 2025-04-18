/**
 * @file actuator_position_node.cpp
 * @author Grayson Arendt
 * @date 2025‑04‑17
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/sockios.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>

/**
 * @class ActuatorPosition
 * @brief SocketCAN interface for Firgelli Automations linear actuator hall sensor.
 *
 * Frame layout (little‑endian, 1 Mbps, ID 0x200):
 *   bytes 0‑3: int32 position [µm] (+ = extend, – = retract)
 *   bytes 4‑7: int32 velocity [µm/s] (unused here, reserved for future use)
 */
class ActuatorPosition : public rclcpp::Node
{
public:
  ActuatorPosition()
  : Node("actuator_position_node")
  {
    position_publisher_ =
      create_publisher<std_msgs::msg::Float64>("/actuator_position", 10);

    declare_parameter("can_interface", "can0");
    get_parameter("can_interface", can_interface_);

    if (!open_can_socket()) {
      RCLCPP_FATAL(get_logger(), "Failed to open %s", can_interface_.c_str());
      rclcpp::shutdown();
      return;
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&ActuatorPosition::read_can_frame, this));
  }

  ~ActuatorPosition() override
  {
    if (can_socket_ >= 0) {close(can_socket_);}
  }

private:
  /**
   * @brief Open a socket to the CAN interface.
   * @return true if successful, false otherwise.
   */
  bool open_can_socket()
  {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0) {return false;}

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {return false;}

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      return false;
    }

    canid_t filter_id = 0x200;
    can_filter filter{filter_id, CAN_SFF_MASK};
    setsockopt(can_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter));
    return true;
  }

  /**
   * @brief Read the CAN frame and publish the position.
   */
  void read_can_frame()
  {
    can_frame f{};
    const ssize_t n = read(can_socket_, &f, sizeof(f));
    if (n != sizeof(f) || f.can_id != 0x200 || f.can_dlc != 8) {return;}

    int32_t pos_um{};
    std::memcpy(&pos_um, &f.data[0], 4);

    std_msgs::msg::Float64 msg;
    msg.data = static_cast<double>(pos_um) / 1000.0; // μm → mm (retain sign)
    position_publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr position_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string can_interface_;
  int can_socket_{-1};
};

/**
 * @brief Main function.
 * Initializes and spins the ActuatorPosition node.
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActuatorPosition>());
  rclcpp::shutdown();
  return 0;
}
