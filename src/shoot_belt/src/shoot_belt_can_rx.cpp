#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

class ShootBeltCanRx final : public rclcpp::Node
{
public:
  ShootBeltCanRx()
  : Node("shoot_belt_can_rx")
  {
    can_rx_topic_ = declare_parameter<std::string>("can_rx_topic", "/can_rx");
    position_topic_ = declare_parameter<std::string>(
      "position_topic", "/shoot_belt/feedback/position");
    mode_topic_ = declare_parameter<std::string>(
      "mode_topic", "/shoot_belt/feedback/control_mode");
    velocity_topic_ = declare_parameter<std::string>(
      "velocity_topic", "/shoot_belt/feedback/velocity");
    fault_topic_ = declare_parameter<std::string>(
      "fault_topic", "/shoot_belt/feedback/fault");

    position_pub_ = create_publisher<std_msgs::msg::Float32>(position_topic_, 100);
    mode_pub_ = create_publisher<std_msgs::msg::UInt8>(mode_topic_, 100);
    velocity_pub_ = create_publisher<std_msgs::msg::Float32>(velocity_topic_, 100);
    fault_pub_ = create_publisher<std_msgs::msg::String>(fault_topic_, 10);
    can_rx_sub_ = create_subscription<can_msgs::msg::Frame>(
      can_rx_topic_, rclcpp::QoS(200),
      std::bind(&ShootBeltCanRx::onFrame, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "CAN RX ready: %s -> position=%s, mode=%s, velocity=%s, fault=%s",
      can_rx_topic_.c_str(), position_topic_.c_str(),
      mode_topic_.c_str(), velocity_topic_.c_str(), fault_topic_.c_str());
  }

private:
  static float decodeFloat32LE(const std::array<uint8_t, 8> & data)
  {
    const uint32_t bits =
      static_cast<uint32_t>(data[0]) |
      (static_cast<uint32_t>(data[1]) << 8U) |
      (static_cast<uint32_t>(data[2]) << 16U) |
      (static_cast<uint32_t>(data[3]) << 24U);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }

  void publishFault(const std::string & text)
  {
    RCLCPP_ERROR(get_logger(), "%s", text.c_str());
    std_msgs::msg::String msg;
    msg.data = text;
    fault_pub_->publish(msg);
  }

  void onFrame(const can_msgs::msg::Frame::SharedPtr frame)
  {
    if (frame->is_extended || frame->is_error || frame->is_rtr) {
      return;
    }

    if (frame->id == 0x210U) {
      // Velocity feedback is assumed to be float32 little-endian [rad/s],
      // matching the encoding used by the other numeric feedback frames.
      if (frame->dlc != 4U) {
        publishFault("CAN RX: ID 0x210 velocity feedback must have DLC=4");
        return;
      }
      const float velocity_rad_s = decodeFloat32LE(frame->data);
      if (!std::isfinite(velocity_rad_s)) {
        publishFault("CAN RX: ID 0x210 velocity feedback is non-finite");
        return;
      }

      std_msgs::msg::Float32 msg;
      msg.data = velocity_rad_s;
      velocity_pub_->publish(msg);
      return;
    }

    if (frame->id == 0x220U) {
      if (frame->dlc != 4U) {
        publishFault("CAN RX: ID 0x220 position feedback must have DLC=4");
        return;
      }
      // STM32 sends signed position [mm] relative to the most recent origin reset as float32.
      const float position_mm = decodeFloat32LE(frame->data);
      if (!std::isfinite(position_mm)) {
        publishFault("CAN RX: ID 0x220 position feedback is non-finite");
        return;
      }

      std_msgs::msg::Float32 msg;
      msg.data = position_mm;
      position_pub_->publish(msg);
      return;
    }

    if (frame->id == 0x230U) {
      if (frame->dlc != 1U || frame->data[0] > 3U) {
        publishFault("CAN RX: ID 0x230 mode feedback must have DLC=1 and mode in [0,3]");
        return;
      }

      std_msgs::msg::UInt8 msg;
      msg.data = frame->data[0];
      mode_pub_->publish(msg);
    }
  }

  std::string can_rx_topic_;
  std::string position_topic_;
  std::string mode_topic_;
  std::string velocity_topic_;
  std::string fault_topic_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_rx_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr position_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShootBeltCanRx>());
  rclcpp::shutdown();
  return 0;
}
