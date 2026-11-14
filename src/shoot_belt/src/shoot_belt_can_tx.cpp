#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shoot_belt/msg/shoot_belt_command.hpp"

class ShootBeltCanTx final : public rclcpp::Node
{
public:
  ShootBeltCanTx()
  : Node("shoot_belt_can_tx")
  {
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/shoot_belt/can_tx_command");
    can_tx_topic_ = declare_parameter<std::string>("can_tx_topic", "/can_tx");

    can_tx_pub_ = create_publisher<can_msgs::msg::Frame>(can_tx_topic_, 20);
    command_sub_ = create_subscription<shoot_belt::msg::ShootBeltCommand>(
      command_topic_, rclcpp::QoS(100),
      std::bind(&ShootBeltCanTx::onCommand, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "CAN TX ready: %s -> %s",
      command_topic_.c_str(), can_tx_topic_.c_str());
  }

private:
  static std::array<uint8_t, 8> encodeFloat32LE(float value)
  {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    std::array<uint8_t, 8> data{};
    data[0] = static_cast<uint8_t>(bits & 0xFFU);
    data[1] = static_cast<uint8_t>((bits >> 8U) & 0xFFU);
    data[2] = static_cast<uint8_t>((bits >> 16U) & 0xFFU);
    data[3] = static_cast<uint8_t>((bits >> 24U) & 0xFFU);
    return data;
  }

  void publishFrame(uint32_t id, uint8_t dlc, const std::array<uint8_t, 8> & data)
  {
    can_msgs::msg::Frame frame;
    frame.header.stamp = now();
    frame.header.frame_id = "shoot_belt";
    frame.id = id;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.is_error = false;
    frame.dlc = dlc;
    frame.data = data;
    can_tx_pub_->publish(frame);
  }

  void onCommand(const shoot_belt::msg::ShootBeltCommand::SharedPtr msg)
  {
    uint32_t can_id = 0U;
    uint8_t dlc = 0U;
    std::array<uint8_t, 8> data{};

    switch (msg->command_type) {
      case shoot_belt::msg::ShootBeltCommand::COMMAND_CURRENT:
        if (!std::isfinite(msg->value)) {
          RCLCPP_ERROR(get_logger(), "Rejecting non-finite current command");
          return;
        }
        can_id = 0x100U;
        dlc = 4U;
        data = encodeFloat32LE(msg->value);
        break;

      case shoot_belt::msg::ShootBeltCommand::COMMAND_POSITION:
        // Transmit the signed target [mm] in the coordinate system established by the latest origin reset.
        if (!std::isfinite(msg->value)) {
          RCLCPP_ERROR(get_logger(), "Rejecting non-finite position command");
          return;
        }
        can_id = 0x120U;
        dlc = 4U;
        data = encodeFloat32LE(msg->value);
        break;

      case shoot_belt::msg::ShootBeltCommand::COMMAND_MODE: {
          if (!std::isfinite(msg->value) || msg->value < 0.0F ||
            msg->value > 3.0F || std::floor(msg->value) != msg->value)
          {
            RCLCPP_ERROR(get_logger(), "Rejecting invalid control mode value %.3f", msg->value);
            return;
          }
          can_id = 0x130U;
          dlc = 1U;
          data[0] = static_cast<uint8_t>(msg->value);
          break;
        }

      case shoot_belt::msg::ShootBeltCommand::COMMAND_ORIGIN_RESET:
        if (!std::isfinite(msg->value) || std::abs(msg->value - 1.0F) > 1.0e-6F) {
          RCLCPP_ERROR(
            get_logger(), "Rejecting origin-reset command unless value is true (1.0)");
          return;
        }
        can_id = 0x140U;
        dlc = 1U;
        data[0] = 1U;
        break;

      default:
        RCLCPP_ERROR(
          get_logger(), "Rejecting unknown command type %u",
          static_cast<unsigned>(msg->command_type));
        return;
    }

    try {
      publishFrame(can_id, dlc, data);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        get_logger(), "Failed to publish CAN frame 0x%03X: %s", can_id, e.what());
    }
  }

  std::string command_topic_;
  std::string can_tx_topic_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_tx_pub_;
  rclcpp::Subscription<shoot_belt::msg::ShootBeltCommand>::SharedPtr command_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShootBeltCanTx>());
  rclcpp::shutdown();
  return 0;
}
