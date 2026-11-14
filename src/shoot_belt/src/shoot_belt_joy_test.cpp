#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sequence_interfaces/action/sequence.hpp"
#include "std_msgs/msg/u_int8.hpp"

class ShootBeltJoyTest final : public rclcpp::Node
{
public:
  using Action = sequence_interfaces::action::Sequence;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  ShootBeltJoyTest()
  : Node("shoot_belt_joy_test")
  {
    joy_topic_ = declare_parameter<std::string>("joy_topic", "/joy");
    operation_state_topic_ = declare_parameter<std::string>(
      "operation_state_topic", "/shoot_belt/operation_state");
    button_index_ = declare_parameter<int>("button_index", 0);

    if (joy_topic_.empty() || operation_state_topic_.empty() || button_index_ < 0) {
      throw std::invalid_argument(
              "joy_topic and operation_state_topic must not be empty; button_index must be >= 0");
    }

    // This test client is intentionally fixed to /kari. It never targets the
    // production /shoot_belt/execute action.
    action_client_ = rclcpp_action::create_client<Action>(this, "/shoot_belt/execute");

    operation_state_sub_ = create_subscription<std_msgs::msg::UInt8>(
      operation_state_topic_,
      rclcpp::QoS(10),
      [this](const std_msgs::msg::UInt8::SharedPtr msg) {
        operation_state_.store(msg->data);
        have_operation_state_.store(true);
      });

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ShootBeltJoyTest::onJoy, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Test Action client ready: joy=%s, button_index=%d, action=/kari, operation=1, state=%s",
      joy_topic_.c_str(), button_index_, operation_state_topic_.c_str());
  }

private:
  static constexpr uint8_t OPERATION_STATE_OK = 2U;

  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const auto index = static_cast<std::size_t>(button_index_);
    if (msg->buttons.size() <= index) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Joy message has %zu buttons; button index %d is unavailable",
        msg->buttons.size(), button_index_);
      return;
    }

    const bool pressed = msg->buttons[index] != 0;
    if (!button_state_initialized_) {
      button_state_initialized_ = true;
      previous_button_pressed_ = pressed;
      RCLCPP_INFO(
        get_logger(),
        "Initial button[%d] state=%s; release then press to trigger",
        button_index_, pressed ? "pressed" : "released");
      return;
    }

    const bool rising_edge = pressed && !previous_button_pressed_;
    previous_button_pressed_ = pressed;
    if (!rising_edge) {
      return;
    }

    if (!have_operation_state_.load()) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring button press: no operation state received on %s",
        operation_state_topic_.c_str());
      return;
    }

    const auto operation_state = operation_state_.load();
    if (operation_state != OPERATION_STATE_OK) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring button press: operation state is %u (2=READY required)",
        static_cast<unsigned>(operation_state));
      return;
    }

    if (goal_pending_or_active_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "Ignoring button press: /kari Goal is pending or active");
      return;
    }

    if (!action_client_->wait_for_action_server(std::chrono::milliseconds(0))) {
      goal_pending_or_active_.store(false);
      RCLCPP_WARN(get_logger(), "Action server /kari is not available; no Goal was sent");
      return;
    }

    Action::Goal goal;
    goal.operation = Action::Goal::OPERATION_FULL_CYCLE;

    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const GoalHandle::SharedPtr goal_handle) {
        if (!goal_handle) {
          goal_pending_or_active_.store(false);
          RCLCPP_WARN(get_logger(), "Action server /kari rejected the Goal");
          return;
        }
        RCLCPP_INFO(get_logger(), "Action server /kari accepted the Goal");
      };

    options.feedback_callback =
      [this](
      GoalHandle::SharedPtr,
      const std::shared_ptr<const Action::Feedback> feedback)
      {
        RCLCPP_DEBUG(
          get_logger(), "/kari feedback: phase=%u progress=%.3f detail=%s",
          static_cast<unsigned>(feedback->phase),
          static_cast<double>(feedback->progress),
          feedback->detail.c_str());
      };

    options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        goal_pending_or_active_.store(false);
        if (!result.result) {
          RCLCPP_ERROR(get_logger(), "/kari returned an empty result");
          return;
        }

        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(
              get_logger(), "/kari completed: success=%s reason=%s",
              result.result->success ? "true" : "false",
              result.result->reason.c_str());
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(get_logger(), "/kari Goal was canceled: %s",
              result.result->reason.c_str());
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "/kari Goal was aborted: %s",
              result.result->reason.c_str());
            break;
          default:
            RCLCPP_ERROR(get_logger(), "/kari returned an unknown result code");
            break;
        }
      };

    try {
      (void)action_client_->async_send_goal(goal, options);
      RCLCPP_INFO(
        get_logger(), "Sent /kari Goal: operation=%u (buttons[%d] rising edge)",
        static_cast<unsigned>(goal.operation), button_index_);
    } catch (const std::exception & e) {
      goal_pending_or_active_.store(false);
      RCLCPP_ERROR(get_logger(), "Failed to send /kari Goal: %s", e.what());
    }
  }

  std::string joy_topic_;
  std::string operation_state_topic_;
  int button_index_{0};

  rclcpp_action::Client<Action>::SharedPtr action_client_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr operation_state_sub_;

  bool button_state_initialized_{false};
  bool previous_button_pressed_{false};
  std::atomic<bool> have_operation_state_{false};
  std::atomic<uint8_t> operation_state_{0U};
  std::atomic<bool> goal_pending_or_active_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ShootBeltJoyTest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
