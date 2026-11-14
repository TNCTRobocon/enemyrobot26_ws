#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>
#include <thread>

#include "shoot_belt/msg/shoot_belt_command.hpp"
#include "shoot_belt/srv/manual_current.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sequence_interfaces/action/sequence.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

using namespace std::chrono_literals;

class ShootBeltController final : public rclcpp::Node
{
public:
  using Action = sequence_interfaces::action::Sequence;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  ShootBeltController()
  : Node("shoot_belt_sequence")
  {
    shoot_current_a_ = declare_parameter<double>("shoot_current_a", 0.0);
    min_shoot_current_a_ = declare_parameter<double>("min_shoot_current_a", 0.1);
    max_shoot_current_a_ = declare_parameter<double>("max_shoot_current_a", 80.0);
    max_acceleration_distance_mm_ = declare_parameter<double>("max_acceleration_distance_mm", 585.0);
    // Used only by the bounded brake-command repetition on interrupt/error paths.
    brake_wait_sec_ = declare_parameter<double>("brake_wait_sec", 0.3);
    brake_velocity_threshold_rad_s_ =
      declare_parameter<double>("brake_velocity_threshold_rad_s", 0.2);
    brake_velocity_stable_time_sec_ =
      declare_parameter<double>("brake_velocity_stable_time_sec", 0.1);
    brake_timeout_sec_ = declare_parameter<double>("brake_timeout_sec", 3.0);
    velocity_feedback_timeout_sec_ =
      declare_parameter<double>("velocity_feedback_timeout_sec", 0.2);
    control_period_ms_ = declare_parameter<int>("control_period_ms", 10);
    feedback_timeout_sec_ = declare_parameter<double>("feedback_timeout_sec", 0.2);
    startup_feedback_timeout_sec_ =
      declare_parameter<double>("startup_feedback_timeout_sec", 1.0);
    mode_confirmation_timeout_sec_ =
      declare_parameter<double>("mode_confirmation_timeout_sec", 0.5);
    acceleration_timeout_sec_ = declare_parameter<double>("acceleration_timeout_sec", 1.5);
    position_tolerance_mm_ = declare_parameter<double>("position_tolerance_mm", 1.415);
    position_stable_time_sec_ = declare_parameter<double>("position_stable_time_sec", 0.1);
    position_return_timeout_sec_ = declare_parameter<double>("position_return_timeout_sec", 5.0);
    origin_return_interval_mm_ = declare_parameter<double>("origin_return_interval_mm", 1125.0);
    origin_reset_timeout_sec_ = declare_parameter<double>("origin_reset_timeout_sec", 0.5);
    origin_reset_retry_interval_sec_ =
      declare_parameter<double>("origin_reset_retry_interval_sec", 0.1);

    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/shoot_belt/can_tx_command");
    feedback_position_topic_ = declare_parameter<std::string>(
      "feedback_position_topic", "/shoot_belt/feedback/position");
    feedback_velocity_topic_ = declare_parameter<std::string>(
      "feedback_velocity_topic", "/shoot_belt/feedback/velocity");
    feedback_mode_topic_ = declare_parameter<std::string>(
      "feedback_mode_topic", "/shoot_belt/feedback/control_mode");
    feedback_fault_topic_ = declare_parameter<std::string>(
      "feedback_fault_topic", "/shoot_belt/feedback/fault");
    stop_topic_ = declare_parameter<std::string>("stop_topic", "/shoot_belt/stop");
    emergency_stop_topic_ = declare_parameter<std::string>(
      "emergency_stop_topic", "/shoot_belt/emergency_stop");
    action_name_ = declare_parameter<std::string>("action_name", "/shoot_belt/execute");
    manual_current_service_name_ = declare_parameter<std::string>(
      "manual_current_service_name", "/shoot_belt/manual_current");
    reset_origin_service_name_ = declare_parameter<std::string>(
      "reset_origin_service_name", "/shoot_belt/reset_origin");
    manual_current_state_topic_ = declare_parameter<std::string>(
      "manual_current_state_topic", "/shoot_belt/manual_current_state");
    operation_state_topic_ = declare_parameter<std::string>(
      "operation_state_topic", "/shoot_belt/operation_state");
    manual_fixed_bucket_current_a_ =
      declare_parameter<double>("manual_fixed_bucket_current_a", 1.0);
    manual_bucket_1_current_a_ =
      declare_parameter<double>("manual_bucket_1_current_a", 1.5);
    manual_bucket_2_current_a_ =
      declare_parameter<double>("manual_bucket_2_current_a", 0.5);
    manual_bucket_3_current_a_ =
      declare_parameter<double>("manual_bucket_3_current_a", 1.0);
    manual_move_current_a_ =
      declare_parameter<double>("manual_move_current_a", 1.0);
    manual_current_step_a_ = declare_parameter<double>("manual_current_step_a", 1.0);
    manual_min_current_a_ = declare_parameter<double>("manual_min_current_a", 0.1);
    manual_max_current_a_ = declare_parameter<double>("manual_max_current_a", 10.0);

    validateParameters();
    control_period_ = std::chrono::milliseconds(control_period_ms_);

    command_pub_ = create_publisher<shoot_belt::msg::ShootBeltCommand>(command_topic_, 20);
    manual_current_state_pub_ = create_publisher<std_msgs::msg::Float32>(
      manual_current_state_topic_, rclcpp::QoS(1).transient_local());
    operation_state_pub_ = create_publisher<std_msgs::msg::UInt8>(
      operation_state_topic_, rclcpp::QoS(1).transient_local());
    operation_state_timer_ = create_wall_timer(
      100ms, std::bind(&ShootBeltController::updateOperationState, this));

    // Feedback and safety callbacks must be able to run while the manual-current
    // service waits for mode confirmation.
    feedback_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    safety_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    origin_reset_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions feedback_options;
    feedback_options.callback_group = feedback_callback_group_;
    rclcpp::SubscriptionOptions safety_options;
    safety_options.callback_group = safety_callback_group_;

    position_sub_ = create_subscription<std_msgs::msg::Float32>(
      feedback_position_topic_, rclcpp::QoS(100),
      std::bind(&ShootBeltController::onPositionFeedback, this, std::placeholders::_1),
      feedback_options);
    velocity_sub_ = create_subscription<std_msgs::msg::Float32>(
      feedback_velocity_topic_, rclcpp::QoS(100),
      std::bind(&ShootBeltController::onVelocityFeedback, this, std::placeholders::_1),
      feedback_options);
    mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
      feedback_mode_topic_, rclcpp::QoS(100),
      std::bind(&ShootBeltController::onModeFeedback, this, std::placeholders::_1),
      feedback_options);
    fault_sub_ = create_subscription<std_msgs::msg::String>(
      feedback_fault_topic_, rclcpp::QoS(10),
      std::bind(&ShootBeltController::onFeedbackFault, this, std::placeholders::_1),
      feedback_options);
    stop_sub_ = create_subscription<std_msgs::msg::Empty>(
      stop_topic_, rclcpp::QoS(10),
      std::bind(&ShootBeltController::onStopRequest, this, std::placeholders::_1),
      safety_options);
    emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      emergency_stop_topic_, rclcpp::QoS(10),
      std::bind(&ShootBeltController::onEmergencyStop, this, std::placeholders::_1),
      safety_options);

    manual_current_service_ = create_service<shoot_belt::srv::ManualCurrent>(
      manual_current_service_name_,
      std::bind(&ShootBeltController::handleManualCurrent, this,
        std::placeholders::_1, std::placeholders::_2));
    reset_origin_service_ = create_service<std_srvs::srv::Trigger>(
      reset_origin_service_name_,
      std::bind(&ShootBeltController::handleResetOrigin, this,
        std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), origin_reset_callback_group_);
    publishManualCurrentState();

    action_server_ = rclcpp_action::create_server<Action>(
      this,
      action_name_,
      std::bind(&ShootBeltController::handleGoal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&ShootBeltController::handleCancel, this, std::placeholders::_1),
      std::bind(&ShootBeltController::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Ready: action=%s, stop=%s, estop=%s, "
      "reset_origin_service=%s, command_topic=%s. shoot_current_a=%.3f A",
      action_name_.c_str(), stop_topic_.c_str(), emergency_stop_topic_.c_str(),
      reset_origin_service_name_.c_str(), command_topic_.c_str(), shoot_current_a_);
    if (shoot_current_a_ < min_shoot_current_a_ ||
      shoot_current_a_ > max_shoot_current_a_)
    {
      RCLCPP_WARN(
        get_logger(),
        "Action goals will be rejected until shoot_current_a is set within [%.3f, %.3f] A",
        min_shoot_current_a_, max_shoot_current_a_);
    }

    RCLCPP_INFO(
      get_logger(),
      "Origin initialization started at node startup; "
      "Action goals remain disabled until it succeeds");
    origin_initialization_active_.store(true);
    initialization_thread_ = std::thread([this]() {
      std::string failure_reason;
      const bool initialized = initializeOrigin(failure_reason);
      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        origin_initialization_active_.store(false);
        operation_state_ = initialized ?
          OperationState::READY : OperationState::RESET_FAILED;
      }
    });
  }

  ~ShootBeltController() override
  {
    shutdown_requested_.store(true);
    if (initialization_thread_.joinable()) {
      initialization_thread_.join();
    }
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  enum class OperationState : uint8_t
  {
    ERROR = 0U,
    RUNNING = 1U,
    READY = 2U,
    CANCELLED = 3U,
    RESETTING = 4U,
    RESET_FAILED = 5U
  };

  enum class Interrupt
  {
    NONE,
    STOP_TOPIC,
    EMERGENCY_STOP,
    ACTION_CANCEL,
    FEEDBACK_MALFORMED,
    FEEDBACK_TIMEOUT,
    MODE_TIMEOUT,
    MODE_MISMATCH,
    ACCELERATION_TIMEOUT,
    POSITION_RETURN_TIMEOUT,
    ORIGIN_RESET_TIMEOUT,
    BRAKE_TIMEOUT,
    SHUTDOWN,
    INTERNAL_ERROR
  };

  struct FeedbackSnapshot
  {
    bool have_position{false};
    bool have_velocity{false};
    bool have_mode{false};
    float position{0.0F};
    float velocity_rad_s{0.0F};
    uint8_t mode{0};
    TimePoint position_time{};
    TimePoint velocity_time{};
    TimePoint mode_time{};
  };

  void validateParameters()
  {
    const auto finite = [](double value) {return std::isfinite(value);};
    if (!finite(min_shoot_current_a_) || !finite(max_shoot_current_a_) ||
      min_shoot_current_a_ < 0.0 || max_shoot_current_a_ <= min_shoot_current_a_)
    {
      throw std::invalid_argument("Invalid min_shoot_current_a / max_shoot_current_a");
    }
    if (!finite(shoot_current_a_) || !finite(max_acceleration_distance_mm_) ||
      max_acceleration_distance_mm_ <= 0.0 ||
      !finite(origin_return_interval_mm_) || origin_return_interval_mm_ <= 0.0)
    {
      throw std::invalid_argument(
        "Current, max_acceleration_distance_mm and origin_return_interval_mm must be finite; "
        "distances must be positive");
    }
    if (!finite(manual_fixed_bucket_current_a_) ||
      !finite(manual_bucket_1_current_a_) || !finite(manual_bucket_2_current_a_) ||
      !finite(manual_bucket_3_current_a_) || !finite(manual_move_current_a_) ||
      !finite(manual_current_step_a_) ||
      !finite(manual_min_current_a_) || !finite(manual_max_current_a_) ||
      manual_current_step_a_ <= 0.0 || manual_min_current_a_ < min_shoot_current_a_ ||
      manual_max_current_a_ <= manual_min_current_a_ ||
      manual_max_current_a_ > max_shoot_current_a_ ||
      manual_fixed_bucket_current_a_ < manual_min_current_a_ ||
      manual_fixed_bucket_current_a_ > manual_max_current_a_ ||
      manual_bucket_1_current_a_ < manual_min_current_a_ ||
      manual_bucket_1_current_a_ > manual_max_current_a_ ||
      manual_bucket_2_current_a_ < manual_min_current_a_ ||
      manual_bucket_2_current_a_ > manual_max_current_a_ ||
      manual_bucket_3_current_a_ < manual_min_current_a_ ||
      manual_bucket_3_current_a_ > manual_max_current_a_ ||
      manual_move_current_a_ < manual_min_current_a_ ||
      manual_move_current_a_ > manual_max_current_a_)
    {
      throw std::invalid_argument(
        "Invalid manual-current settings: check fixed value, step, and bounds");
    }
    if (!finite(brake_wait_sec_) || brake_wait_sec_ < 0.0 ||
      control_period_ms_ <= 0 ||
      !finite(feedback_timeout_sec_) || feedback_timeout_sec_ <= 0.0 ||
      !finite(startup_feedback_timeout_sec_) || startup_feedback_timeout_sec_ <= 0.0 ||
      !finite(mode_confirmation_timeout_sec_) || mode_confirmation_timeout_sec_ <= 0.0 ||
      !finite(acceleration_timeout_sec_) || acceleration_timeout_sec_ <= 0.0 ||
      !finite(position_tolerance_mm_) || position_tolerance_mm_ <= 0.0 ||
      !finite(position_stable_time_sec_) || position_stable_time_sec_ < 0.0 ||
      !finite(position_return_timeout_sec_) || position_return_timeout_sec_ <= 0.0 ||
      !finite(origin_reset_timeout_sec_) || origin_reset_timeout_sec_ <= 0.0 ||
      !finite(origin_reset_retry_interval_sec_) || origin_reset_retry_interval_sec_ <= 0.0 ||
      !finite(brake_velocity_threshold_rad_s_) || brake_velocity_threshold_rad_s_ < 0.0 ||
      !finite(brake_velocity_stable_time_sec_) || brake_velocity_stable_time_sec_ < 0.0 ||
      !finite(brake_timeout_sec_) || brake_timeout_sec_ <= 0.0 ||
      !finite(velocity_feedback_timeout_sec_) || velocity_feedback_timeout_sec_ <= 0.0)
    {
      throw std::invalid_argument("One or more time/position parameters are outside valid ranges");
    }
  }

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Action::Goal> goal)
  {
    if (goal->operation != Action::Goal::OPERATION_FULL_CYCLE) {
      RCLCPP_WARN(
        get_logger(), "Rejecting Action: operation must be OPERATION_FULL_CYCLE (1), got %u",
        static_cast<unsigned>(goal->operation));
      return rclcpp_action::GoalResponse::REJECT;
    }

    const auto feedback = getFeedbackSnapshot();
    std::lock_guard<std::mutex> lock(goal_mutex_);

    // The state check and RUNNING transition share the same mutex. Concurrent
    // Goal callbacks therefore cannot both pass the READY check.
    if (operation_state_ != OperationState::READY) {
      RCLCPP_WARN(
        get_logger(), "Rejecting Action: operation state is %u (READY=2 required)",
        static_cast<unsigned>(operation_state_));
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!initialization_complete_.load() || origin_initialization_active_.load() ||
      startup_abort_requested_.load() || shutdown_requested_.load() ||
      emergency_stop_active_.load() || emergency_stop_event_.load() ||
      feedback_fault_.load() || stop_requested_.load() || cancel_requested_.load() ||
      !feedbackIsFresh(feedback))
    {
      operation_state_ = OperationState::ERROR;
      RCLCPP_ERROR(
        get_logger(),
        "Rejecting Action: initialization, safety input, feedback, or controller state is not ready");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (active_.load()) {
      operation_state_ = OperationState::ERROR;
      RCLCPP_WARN(get_logger(), "Rejecting Action: another Action is already running");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!std::isfinite(shoot_current_a_) ||
      shoot_current_a_ < min_shoot_current_a_ ||
      shoot_current_a_ > max_shoot_current_a_)
    {
      operation_state_ = OperationState::ERROR;
      RCLCPP_ERROR(
        get_logger(),
        "Rejecting Action: shoot_current_a %.3f A is outside [%.3f, %.3f] A",
        shoot_current_a_, min_shoot_current_a_, max_shoot_current_a_);
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Freeze the current setpoint for this Action. GUI changes made after this
    // point are retained for the next shot and do not alter this one.
    active_shoot_current_a_ = shoot_current_a_;
    active_.store(true);
    operation_state_ = OperationState::RUNNING;

    // Publish RUNNING immediately so belt_reload and the GUI close the
    // reload interlock as soon as this shooting Action is accepted. The
    // periodic publisher remains in place for latched-state refresh.
    std_msgs::msg::UInt8 operation_state_message;
    operation_state_message.data =
      static_cast<uint8_t>(operation_state_);
    operation_state_pub_->publish(operation_state_message);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void publishManualCurrentState()
  {
    std_msgs::msg::Float32 state;
    state.data = static_cast<float>(shoot_current_a_);
    manual_current_state_pub_->publish(state);
  }

  void updateOperationState()
  {
    const auto feedback = getFeedbackSnapshot();
    std_msgs::msg::UInt8 message;

    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (operation_state_ == OperationState::READY) {
        const bool current_is_valid = std::isfinite(shoot_current_a_) &&
          shoot_current_a_ >= min_shoot_current_a_ &&
          shoot_current_a_ <= max_shoot_current_a_;
        const bool runtime_is_safe =
          initialization_complete_.load() &&
          !active_.load() &&
          !emergency_stop_active_.load() &&
          !feedback_fault_.load() &&
          !emergency_stop_event_.load() &&
          !origin_initialization_active_.load() &&
          !startup_abort_requested_.load() &&
          !stop_requested_.load() &&
          !cancel_requested_.load() &&
          !shutdown_requested_.load() &&
          current_is_valid &&
          feedbackIsFresh(feedback);

        // Once READY is lost, recovery is explicit: only a successful RESET
        // may transition the controller back to READY.
        if (!runtime_is_safe) {
          operation_state_ = OperationState::ERROR;
        }
      }

      message.data = static_cast<uint8_t>(operation_state_);
    }

    operation_state_pub_->publish(message);
  }

  void handleManualCurrent(
    const std::shared_ptr<shoot_belt::srv::ManualCurrent::Request> request,
    std::shared_ptr<shoot_belt::srv::ManualCurrent::Response> response)
  {
    // This service changes only the stored shot-current setpoint. It deliberately
    // does not publish current or mode commands, and does not require motor
    // feedback: a settings change must not itself move the mechanism.
    std::lock_guard<std::mutex> manual_lock(manual_current_mutex_);
    std::lock_guard<std::mutex> goal_lock(goal_mutex_);

    response->success = false;
    response->current_a = static_cast<float>(shoot_current_a_);

    double target_current_a = shoot_current_a_;
    switch (request->operation) {
      case shoot_belt::srv::ManualCurrent::Request::SET_FIXED_BUCKET:
        // Keep the legacy operation ID and its configured value compatible.
        target_current_a = manual_fixed_bucket_current_a_;
        break;
      case shoot_belt::srv::ManualCurrent::Request::SET_FIXED_BUCKET_1:
        target_current_a = manual_bucket_1_current_a_;
        break;
      case shoot_belt::srv::ManualCurrent::Request::SET_FIXED_BUCKET_2:
        target_current_a = manual_bucket_2_current_a_;
        break;
      case shoot_belt::srv::ManualCurrent::Request::SET_FIXED_BUCKET_3:
        target_current_a = manual_bucket_3_current_a_;
        break;
      case shoot_belt::srv::ManualCurrent::Request::SET_MOVE:
        target_current_a = manual_move_current_a_;
        break;
      case shoot_belt::srv::ManualCurrent::Request::INCREASE:
        target_current_a += manual_current_step_a_;
        break;
      case shoot_belt::srv::ManualCurrent::Request::DECREASE:
        target_current_a -= manual_current_step_a_;
        break;
      default:
        response->message = "Unknown shot-current setting operation";
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
    }

    target_current_a = std::clamp(
      target_current_a, manual_min_current_a_, manual_max_current_a_);
    shoot_current_a_ = target_current_a;

    publishManualCurrentState();
    response->success = true;
    response->current_a = static_cast<float>(shoot_current_a_);
    response->message =
      "Shot-current setpoint updated; no motor command was sent";
    RCLCPP_INFO(
      get_logger(), "Shot-current setpoint updated to %.3f A (setting only)",
      shoot_current_a_);
  }

  void handleResetOrigin(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    response->success = false;

    {
      // Reserve RESETTING while holding the same mutex used by handleGoal.
      // Repeated RESET requests and Action Goals cannot race this transition.
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (shutdown_requested_.load() || !rclcpp::ok()) {
        response->message = "Controller is shutting down";
      } else if (operation_state_ == OperationState::RESETTING) {
        response->message = "Origin initialization is already in progress";
      } else if (active_.load()) {
        response->message = "Cannot reset origin while an Action is active";
      } else if (emergency_stop_active_.load()) {
        response->message = "Cannot reset origin while emergency stop is active";
      } else if (operation_state_ != OperationState::CANCELLED &&
        operation_state_ != OperationState::ERROR &&
        operation_state_ != OperationState::RESET_FAILED)
      {
        response->message =
          "RESET is accepted only in CANCELLED, ERROR, or RESET_FAILED state";
      } else {
        operation_state_ = OperationState::RESETTING;
        origin_initialization_active_.store(true);
        initialization_complete_.store(false);
        startup_abort_requested_.store(false);
        stop_requested_.store(false);
        cancel_requested_.store(false);
        emergency_stop_event_.store(false);

        // A latched feedback error may be recoverable after operator
        // intervention. Clear it for the retry; any continuing fault callback
        // will latch it again and make initialization fail safely.
        feedback_fault_.store(false);
        {
          std::lock_guard<std::mutex> fault_lock(fault_mutex_);
          feedback_fault_text_.clear();
        }
      }
    }

    if (!response->message.empty()) {
      RCLCPP_WARN(get_logger(), "Origin reset rejected: %s", response->message.c_str());
      return;
    }

    RCLCPP_WARN(get_logger(), "Manual origin initialization requested");
    std::string failure_reason;
    const bool initialized = initializeOrigin(failure_reason);
    response->success = initialized;
    response->message = initialized ?
      "Origin initialization completed; position mode is holding 0 mm" :
      ("Origin initialization failed: " + failure_reason);

    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      origin_initialization_active_.store(false);
      operation_state_ = initialized ?
        OperationState::READY : OperationState::RESET_FAILED;
    }

    if (response->success) {
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    }
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    cancel_requested_.store(true);
    RCLCPP_WARN(get_logger(), "Action cancel received; routing through brake sequence");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    worker_thread_ = std::thread(
      [this, goal_handle]() {execute(goal_handle);});
  }

  void onStopRequest(const std_msgs::msg::Empty::SharedPtr)
  {
    bool action_was_active = false;
    bool ready_state_cancelled = false;
    {
      // Serialize stop arrival against the Action terminal transition.
      std::lock_guard<std::mutex> lock(goal_mutex_);
      action_was_active = active_.load();
      if (action_was_active) {
        stop_requested_.store(true);
      } else if (origin_initialization_active_.load() ||
        !initialization_complete_.load())
      {
        startup_abort_requested_.store(true);
      } else if (operation_state_ == OperationState::READY) {
        // An idle CANCEL must also latch the state. Otherwise the controller
        // remains READY and the GUI keeps showing OK with RESET disabled.
        operation_state_ = OperationState::CANCELLED;
        ready_state_cancelled = true;
      }
    }

    if (action_was_active) {
      RCLCPP_ERROR(get_logger(), "Dedicated stop request received during Action");
    } else if (ready_state_cancelled) {
      RCLCPP_WARN(
        get_logger(), "Stop request while READY: latching CANCELLED and commanding brake mode");
    } else {
      // Preserve ERROR, CANCELLED, RESETTING, and RESET_FAILED while still
      // issuing the brake command for an idle/startup stop request.
      RCLCPP_WARN(
        get_logger(), "Stop request while idle/startup: commanding brake mode; operation state preserved");
    }

    // Make the brake request immediately; the Action worker repeats it while
    // completing the interrupted sequence.
    publishMode(3U);
    publishCurrent(0.0F);
  }

  void onEmergencyStop(const std_msgs::msg::Bool::SharedPtr msg)
  {
    {
      // Keep emergency-stop event ordering consistent with Action completion.
      std::lock_guard<std::mutex> lock(goal_mutex_);
      emergency_stop_active_.store(msg->data);
      if (msg->data) {
        if (active_.load()) {
          emergency_stop_event_.store(true);
        } else if (origin_initialization_active_.load() ||
          !initialization_complete_.load())
        {
          startup_abort_requested_.store(true);
        }
      }
    }

    if (msg->data) {
      RCLCPP_FATAL(get_logger(), "Emergency-stop input asserted; commanding brake mode now");
      publishMode(3U);
      publishCurrent(0.0F);
    } else {
      RCLCPP_INFO(get_logger(), "Emergency-stop input cleared; no interrupted Action is resumed");
    }
  }

  void markFeedbackFault(const std::string & description)
  {
    // Serialize the fault latch with the Action terminal transition. Keep the
    // lock order goal_mutex_ -> fault_mutex_, matching the RESET callback.
    bool should_command_brake = false;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      should_command_brake =
        active_.load() || origin_initialization_active_.load() ||
        !initialization_complete_.load();
      feedback_fault_.store(true);
      {
        std::lock_guard<std::mutex> fault_lock(fault_mutex_);
        feedback_fault_text_ = description;
      }
    }

    if (!should_command_brake) {
      return;
    }
    // Propagate the stop command immediately; the worker repeats it during shutdown.
    publishMode(3U);
    publishCurrent(0.0F);
  }

  void onPositionFeedback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      markFeedbackFault("Decoded position feedback is non-finite");
      return;
    }
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_.position = msg->data;
    feedback_.have_position = true;
    feedback_.position_time = Clock::now();
  }

  void onVelocityFeedback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    if (!std::isfinite(msg->data)) {
      markFeedbackFault("Decoded velocity feedback is non-finite");
      return;
    }
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_.velocity_rad_s = msg->data;
    feedback_.have_velocity = true;
    feedback_.velocity_time = Clock::now();
  }

  void onModeFeedback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    if (msg->data > 3U) {
      markFeedbackFault("Decoded control-mode feedback is outside [0,3]");
      return;
    }
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    feedback_.mode = msg->data;
    feedback_.have_mode = true;
    feedback_.mode_time = Clock::now();
  }

  void onFeedbackFault(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!msg->data.empty()) {
      markFeedbackFault(msg->data);
    }
  }

  FeedbackSnapshot getFeedbackSnapshot()
  {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    return feedback_;
  }

  bool feedbackIsFresh(const FeedbackSnapshot & f) const
  {
    if (!f.have_position || !f.have_mode) {
      return false;
    }
    const auto now = Clock::now();
    const auto limit = std::chrono::duration<double>(feedback_timeout_sec_);
    return (now - f.position_time) <= limit && (now - f.mode_time) <= limit;
  }

  Interrupt detectInterrupt(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    if (shutdown_requested_.load() || !rclcpp::ok()) {
      return Interrupt::SHUTDOWN;
    }
    if (emergency_stop_event_.load() || emergency_stop_active_.load()) {
      return Interrupt::EMERGENCY_STOP;
    }
    if (goal_handle->is_canceling() || cancel_requested_.load()) {
      return Interrupt::ACTION_CANCEL;
    }
    if (stop_requested_.load()) {
      return Interrupt::STOP_TOPIC;
    }
    if (feedback_fault_.load()) {
      return Interrupt::FEEDBACK_MALFORMED;
    }
    if (!feedbackIsFresh(getFeedbackSnapshot())) {
      return Interrupt::FEEDBACK_TIMEOUT;
    }
    return Interrupt::NONE;
  }

  void publishCommand(uint8_t command_type, float value)
  {
    std::lock_guard<std::mutex> command_lock(command_publish_mutex_);
    const bool stop_latched =
      stop_requested_.load() || cancel_requested_.load() ||
      emergency_stop_event_.load() || emergency_stop_active_.load() ||
      startup_abort_requested_.load() || feedback_fault_.load() ||
      shutdown_requested_.load();
    if (stop_latched) {
      const bool safe_zero_current =
        command_type == shoot_belt::msg::ShootBeltCommand::COMMAND_CURRENT &&
        std::abs(value) <= 1.0e-6F;
      const bool brake_mode =
        command_type == shoot_belt::msg::ShootBeltCommand::COMMAND_MODE &&
        std::abs(value - 3.0F) <= 1.0e-6F;
      // Once a stop/fault is latched, block position commands, nonzero current,
      // and any mode other than brake. This prevents a racing control-loop
      // iteration from overwriting the stop command.
      if (!safe_zero_current && !brake_mode) {
        return;
      }
    }

    shoot_belt::msg::ShootBeltCommand command;
    command.command_type = command_type;
    command.value = value;
    command_pub_->publish(command);
  }

  void publishCurrent(float ampere)
  {
    publishCommand(shoot_belt::msg::ShootBeltCommand::COMMAND_CURRENT, ampere);
  }

  // Position commands use millimeters throughout the ROS 2 <-> STM32 protocol.
  void publishPosition(float position_mm)
  {
    publishCommand(shoot_belt::msg::ShootBeltCommand::COMMAND_POSITION, position_mm);
  }

  void publishMode(uint8_t mode)
  {
    publishCommand(shoot_belt::msg::ShootBeltCommand::COMMAND_MODE, static_cast<float>(mode));
  }

  void publishOriginReset()
  {
    publishCommand(
      shoot_belt::msg::ShootBeltCommand::COMMAND_ORIGIN_RESET, 1.0F);
  }

  void sendActionFeedback(
    const std::shared_ptr<GoalHandle> & goal_handle,
    uint8_t phase, float progress, const std::string & detail)
  {
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->phase = phase;
    feedback->progress = std::clamp(progress, 0.0F, 1.0F);
    feedback->active_actuator = "belt_motor";
    feedback->detail = detail;
    goal_handle->publish_feedback(feedback);
  }

  bool waitForInitialFeedback(
    const std::shared_ptr<GoalHandle> & goal_handle, Interrupt & reason)
  {
    const auto deadline = Clock::now() +
      std::chrono::duration<double>(startup_feedback_timeout_sec_);
    while (rclcpp::ok() && Clock::now() < deadline) {
      if (shutdown_requested_.load()) {
        reason = Interrupt::SHUTDOWN;
        return false;
      }
      if (emergency_stop_event_.load() || emergency_stop_active_.load()) {
        reason = Interrupt::EMERGENCY_STOP;
        return false;
      }
      if (goal_handle->is_canceling() || cancel_requested_.load()) {
        reason = Interrupt::ACTION_CANCEL;
        return false;
      }
      if (stop_requested_.load()) {
        reason = Interrupt::STOP_TOPIC;
        return false;
      }
      if (feedback_fault_.load()) {
        reason = Interrupt::FEEDBACK_MALFORMED;
        return false;
      }
      if (feedbackIsFresh(getFeedbackSnapshot())) {
        reason = Interrupt::NONE;
        return true;
      }
      sendActionFeedback(goal_handle, 1U, 0.0F, "Waiting for fresh 0x220/0x230 feedback");
      std::this_thread::sleep_for(control_period_);
    }
    reason = rclcpp::ok() ? Interrupt::FEEDBACK_TIMEOUT : Interrupt::SHUTDOWN;
    return false;
  }

  bool commandModeAndWait(
    uint8_t target_mode,
    bool resend_position,
    float target_position,
    const std::shared_ptr<GoalHandle> & goal_handle,
    uint8_t phase,
    Interrupt & reason)
  {
    const auto deadline = Clock::now() +
      std::chrono::duration<double>(mode_confirmation_timeout_sec_);
    while (rclcpp::ok() && Clock::now() < deadline) {
      reason = detectInterrupt(goal_handle);
      if (reason != Interrupt::NONE) {
        return false;
      }
      if (resend_position) {
        publishPosition(target_position);
      }
      publishMode(target_mode);
      const auto feedback = getFeedbackSnapshot();
      if (feedback.have_mode && feedback.mode == target_mode &&
        (Clock::now() - feedback.mode_time) <=
        std::chrono::duration<double>(feedback_timeout_sec_))
      {
        reason = Interrupt::NONE;
        return true;
      }
      sendActionFeedback(
        goal_handle, phase, 0.1F,
        "Waiting for control-mode feedback " + std::to_string(target_mode));
      std::this_thread::sleep_for(control_period_);
    }
    reason = rclcpp::ok() ? Interrupt::MODE_TIMEOUT : Interrupt::SHUTDOWN;
    return false;
  }

  bool waitWithSafetyChecks(
    double duration_sec,
    uint8_t phase,
    const std::shared_ptr<GoalHandle> & goal_handle,
    Interrupt & reason,
    bool repeat_brake_mode = false,
    int expected_mode = -1)
  {
    const auto deadline = Clock::now() + std::chrono::duration<double>(duration_sec);
    while (rclcpp::ok() && Clock::now() < deadline) {
      reason = detectInterrupt(goal_handle);
      if (reason != Interrupt::NONE) {
        return false;
      }
      if (expected_mode >= 0 && getFeedbackSnapshot().mode !=
        static_cast<uint8_t>(expected_mode))
      {
        reason = Interrupt::MODE_MISMATCH;
        return false;
      }
      if (repeat_brake_mode) {
        publishMode(3U);
      }
      sendActionFeedback(goal_handle, phase, 0.75F, "Brake wait");
      std::this_thread::sleep_for(control_period_);
    }
    if (!rclcpp::ok()) {
      reason = Interrupt::SHUTDOWN;
      return false;
    }
    reason = Interrupt::NONE;
    return true;
  }

  void commandBrakeImmediately()
  {
    // Brake mode first; clear the current setpoint immediately afterward.
    publishMode(3U);
    publishCurrent(0.0F);
    const auto deadline = Clock::now() + std::chrono::duration<double>(brake_wait_sec_);
    while (rclcpp::ok() && Clock::now() < deadline) {
      publishMode(3U);
      publishCurrent(0.0F);
      std::this_thread::sleep_for(control_period_);
    }
  }

  bool startupCanContinue(Interrupt & reason)
  {
    if (shutdown_requested_.load() || !rclcpp::ok()) {
      reason = Interrupt::SHUTDOWN;
      return false;
    }
    if (emergency_stop_active_.load() || emergency_stop_event_.load()) {
      reason = Interrupt::EMERGENCY_STOP;
      return false;
    }
    if (startup_abort_requested_.load()) {
      reason = Interrupt::STOP_TOPIC;
      return false;
    }
    if (feedback_fault_.load()) {
      reason = Interrupt::FEEDBACK_MALFORMED;
      return false;
    }
    reason = Interrupt::NONE;
    return true;
  }

  bool startupCommandModeAndWait(
    uint8_t target_mode,
    bool resend_position,
    float target_position,
    Interrupt & reason)
  {
    const auto deadline = Clock::now() +
      std::chrono::duration<double>(mode_confirmation_timeout_sec_);
    while (rclcpp::ok() && Clock::now() < deadline) {
      if (!startupCanContinue(reason)) {
        return false;
      }
      if (resend_position) {
        publishPosition(target_position);
      }
      publishMode(target_mode);
      const auto feedback = getFeedbackSnapshot();
      if (feedbackIsFresh(feedback) && feedback.mode == target_mode) {
        reason = Interrupt::NONE;
        return true;
      }
      std::this_thread::sleep_for(control_period_);
    }
    reason = rclcpp::ok() ? Interrupt::MODE_TIMEOUT : Interrupt::SHUTDOWN;
    return false;
  }

  bool waitForStartupPositionStable(
    float target_position,
    uint8_t expected_mode,
    Interrupt & reason)
  {
    const auto deadline = Clock::now() +
      std::chrono::duration<double>(position_return_timeout_sec_);
    TimePoint last_position_time{};
    TimePoint stable_start{};
    bool inside_tolerance = false;

    while (rclcpp::ok() && Clock::now() < deadline) {
      if (!startupCanContinue(reason)) {
        return false;
      }
      const auto feedback = getFeedbackSnapshot();
      if (!feedbackIsFresh(feedback)) {
        std::this_thread::sleep_for(control_period_);
        continue;
      }
      if (feedback.mode != expected_mode) {
        reason = Interrupt::MODE_MISMATCH;
        return false;
      }

      publishPosition(target_position);
      if (feedback.position_time != last_position_time) {
        last_position_time = feedback.position_time;
        const double error_mm = std::abs(
          static_cast<double>(feedback.position) - static_cast<double>(target_position));
        if (error_mm <= position_tolerance_mm_) {
          if (!inside_tolerance) {
            stable_start = Clock::now();
            inside_tolerance = true;
          } else if (Clock::now() - stable_start >=
            std::chrono::duration<double>(position_stable_time_sec_))
          {
            reason = Interrupt::NONE;
            return true;
          }
        } else {
          inside_tolerance = false;
        }
      }
      std::this_thread::sleep_for(control_period_);
    }

    reason = rclcpp::ok() ? Interrupt::POSITION_RETURN_TIMEOUT : Interrupt::SHUTDOWN;
    return false;
  }

  bool waitForOriginResetFeedback(
    const std::shared_ptr<GoalHandle> & goal_handle,
    TimePoint reset_request_time,
    Interrupt & reason)
  {
    const auto deadline = Clock::now() +
      std::chrono::duration<double>(origin_reset_timeout_sec_);
    const auto retry_interval =
      std::chrono::duration_cast<Clock::duration>(
      std::chrono::duration<double>(origin_reset_retry_interval_sec_));
    auto next_reset_request_time = reset_request_time + retry_interval;
    TimePoint last_position_time{};
    TimePoint stable_start{};
    bool inside_tolerance = false;
    unsigned int retry_count = 0U;

    while (rclcpp::ok() && Clock::now() < deadline) {
      if (goal_handle) {
        reason = detectInterrupt(goal_handle);
        if (reason != Interrupt::NONE) {
          return false;
        }
      } else if (!startupCanContinue(reason)) {
        return false;
      }

      const auto feedback = getFeedbackSnapshot();
      if (feedbackIsFresh(feedback)) {
        if (feedback.mode != 3U) {
          reason = Interrupt::MODE_MISMATCH;
          return false;
        }

        // Only feedback newer than the latest reset request can confirm it.
        // Stability is evaluated only when a new position sample arrives.
        if (feedback.position_time > reset_request_time &&
          feedback.position_time != last_position_time)
        {
          last_position_time = feedback.position_time;
          const double error_mm = std::abs(static_cast<double>(feedback.position));
          if (error_mm <= position_tolerance_mm_) {
            if (!inside_tolerance) {
              stable_start = Clock::now();
              inside_tolerance = true;
            } else if (Clock::now() - stable_start >=
              std::chrono::duration<double>(position_stable_time_sec_))
            {
              RCLCPP_INFO(
                get_logger(),
                "Origin reset confirmed near 0 mm after %u retries",
                retry_count);
              reason = Interrupt::NONE;
              return true;
            }
          } else {
            inside_tolerance = false;
          }
        }
      } else {
        // Without fresh position feedback, zero cannot be confirmed. Resume
        // periodic origin-reset requests until fresh feedback is available.
        inside_tolerance = false;
      }

      const auto now = Clock::now();
      if (!inside_tolerance && now >= next_reset_request_time) {
        reset_request_time = now;
        publishOriginReset();
        ++retry_count;
        next_reset_request_time = reset_request_time + retry_interval;
      }

      if (goal_handle) {
        sendActionFeedback(
          goal_handle, 6U, 0.94F,
          "Waiting for fresh position feedback near 0 mm after origin reset; retries=" +
          std::to_string(retry_count));
      }
      std::this_thread::sleep_for(control_period_);
    }

    reason = rclcpp::ok() ? Interrupt::ORIGIN_RESET_TIMEOUT : Interrupt::SHUTDOWN;
    return false;
  }

  bool waitForPositionTarget(
    const std::shared_ptr<GoalHandle> & goal_handle,
    float target_position,
    const std::string & description,
    Interrupt & reason)
  {
    const auto deadline = Clock::now() +
      std::chrono::duration<double>(position_return_timeout_sec_);
    TimePoint stable_start{};
    bool inside_tolerance = false;

    while (rclcpp::ok()) {
      reason = detectInterrupt(goal_handle);
      if (reason != Interrupt::NONE) {
        return false;
      }
      if (Clock::now() >= deadline) {
        reason = Interrupt::POSITION_RETURN_TIMEOUT;
        return false;
      }

      const auto feedback = getFeedbackSnapshot();
      if (feedback.mode != 2U) {
        reason = Interrupt::MODE_MISMATCH;
        return false;
      }
      const double error_mm = std::abs(
        static_cast<double>(feedback.position) - static_cast<double>(target_position));
      publishPosition(target_position);
      sendActionFeedback(
        goal_handle, 5U, 0.88F,
        description + "; absolute error=" + std::to_string(error_mm) + " mm");

      if (error_mm <= position_tolerance_mm_) {
        if (!inside_tolerance) {
          stable_start = Clock::now();
          inside_tolerance = true;
        } else if (Clock::now() - stable_start >=
          std::chrono::duration<double>(position_stable_time_sec_))
        {
          reason = Interrupt::NONE;
          return true;
        }
      } else {
        inside_tolerance = false;
      }
      std::this_thread::sleep_for(control_period_);
    }
    reason = Interrupt::SHUTDOWN;
    return false;
  }

  bool waitForBrakeVelocity(
    const std::shared_ptr<GoalHandle> & goal_handle,
    uint8_t phase,
    Interrupt & reason)
  {
    const auto deadline = Clock::now() +
      std::chrono::duration<double>(brake_timeout_sec_);
    TimePoint last_velocity_time{};
    TimePoint stable_start{};
    bool inside_threshold = false;

    while (rclcpp::ok() && Clock::now() < deadline) {
      if (goal_handle) {
        reason = detectInterrupt(goal_handle);
        if (reason != Interrupt::NONE) {
          return false;
        }
      } else if (!startupCanContinue(reason)) {
        return false;
      }

      const auto feedback = getFeedbackSnapshot();
      if (!feedbackIsFresh(feedback)) {
        reason = Interrupt::FEEDBACK_TIMEOUT;
        return false;
      }
      if (feedback.mode != 3U) {
        reason = Interrupt::MODE_MISMATCH;
        return false;
      }

      const auto now = Clock::now();
      if (feedback.have_velocity) {
        if ((now - feedback.velocity_time) >
          std::chrono::duration<double>(velocity_feedback_timeout_sec_))
        {
          reason = Interrupt::FEEDBACK_TIMEOUT;
          return false;
        }

        // Evaluate the stability window only when a new velocity sample arrives.
        // A frozen near-zero value cannot satisfy the stop condition.
        if (feedback.velocity_time != last_velocity_time) {
          last_velocity_time = feedback.velocity_time;
          if (std::abs(static_cast<double>(feedback.velocity_rad_s)) <=
            brake_velocity_threshold_rad_s_)
          {
            if (!inside_threshold) {
              stable_start = now;
              inside_threshold = true;
            } else if ((now - stable_start) >=
              std::chrono::duration<double>(brake_velocity_stable_time_sec_))
            {
              reason = Interrupt::NONE;
              RCLCPP_INFO(
                get_logger(),
                "Brake complete: velocity %.4f rad/s remained within +/-%.4f rad/s",
                feedback.velocity_rad_s, brake_velocity_threshold_rad_s_);
              return true;
            }
          } else {
            inside_threshold = false;
          }
        }
      }

      publishCurrent(0.0F);
      publishMode(3U);
      if (goal_handle) {
        const std::string velocity_detail = feedback.have_velocity ?
          ("Braking: velocity=" + std::to_string(feedback.velocity_rad_s) + " rad/s") :
          "Braking: waiting for first velocity feedback";
        sendActionFeedback(goal_handle, phase, 0.72F, velocity_detail);
      }
      std::this_thread::sleep_for(control_period_);
    }

    reason = rclcpp::ok() ? Interrupt::BRAKE_TIMEOUT : Interrupt::SHUTDOWN;
    return false;
  }

  bool initializeOrigin(std::string & failure_reason)
  {
    Interrupt reason = Interrupt::NONE;
    try {
      const auto feedback_deadline = Clock::now() +
        std::chrono::duration<double>(startup_feedback_timeout_sec_);
      while (rclcpp::ok() && Clock::now() < feedback_deadline) {
        if (!startupCanContinue(reason)) {
          throw reason;
        }
        if (feedbackIsFresh(getFeedbackSnapshot())) {
          break;
        }
        std::this_thread::sleep_for(control_period_);
      }
      if (!feedbackIsFresh(getFeedbackSnapshot())) {
        reason = rclcpp::ok() ? Interrupt::FEEDBACK_TIMEOUT : Interrupt::SHUTDOWN;
        throw reason;
      }

      RCLCPP_INFO(get_logger(), "Origin initialization: commanding brake mode before origin reset");
      publishCurrent(0.0F);
      if (!startupCommandModeAndWait(3U, false, 0.0F, reason)) {
        throw reason;
      }

      if (!waitForBrakeVelocity(nullptr, 1U, reason)) {
        throw reason;
      }

      // Preload the post-reset target while brake mode is active, avoiding a
      // stale position target after the STM32 coordinate is reset.
      publishPosition(0.0F);
      const auto target_preload_deadline = Clock::now() + control_period_;
      while (rclcpp::ok() && Clock::now() < target_preload_deadline) {
        if (!startupCanContinue(reason)) {
          throw reason;
        }
        const auto feedback = getFeedbackSnapshot();
        if (!feedbackIsFresh(feedback) || feedback.mode != 3U) {
          reason = feedbackIsFresh(feedback) ?
            Interrupt::MODE_MISMATCH : Interrupt::FEEDBACK_TIMEOUT;
          throw reason;
        }
        publishMode(3U);
        std::this_thread::sleep_for(control_period_);
      }

      const auto reset_request_time = Clock::now();
      publishOriginReset();
      if (!waitForOriginResetFeedback(nullptr, reset_request_time, reason)) {
        throw reason;
      }

      if (!startupCommandModeAndWait(2U, true, 0.0F, reason)) {
        throw reason;
      }
      if (!waitForStartupPositionStable(0.0F, 2U, reason)) {
        throw reason;
      }
      if (!startupCanContinue(reason)) {
        throw reason;
      }

      initialization_complete_.store(true);
      RCLCPP_INFO(
        get_logger(),
        "Origin initialization complete; holding 0 mm in position mode");
      failure_reason.clear();
      return true;
    } catch (Interrupt caught) {
      reason = caught;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Origin initialization exception: %s", e.what());
      reason = Interrupt::INTERNAL_ERROR;
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Unknown origin initialization exception");
      reason = Interrupt::INTERNAL_ERROR;
    }

    publishMode(3U);
    publishCurrent(0.0F);
    failure_reason = interruptText(reason);
    RCLCPP_ERROR(
      get_logger(),
      "Origin initialization failed: %s. Actions remain disabled until initialization succeeds.",
      failure_reason.c_str());
    return false;
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto result = std::make_shared<Action::Result>();
    Interrupt reason = Interrupt::NONE;
    bool action_succeeded = false;
    float start_position = 0.0F;
    const double current_for_this_action = active_shoot_current_a_;
    try {
      sendActionFeedback(goal_handle, 1U, 0.0F, "Checking feedback and capturing start position");
      if (!waitForInitialFeedback(goal_handle, reason)) {
        throw reason;
      }

      start_position = getFeedbackSnapshot().position;

      // Ensure no stale non-zero current setpoint is active before entering current mode.
      publishCurrent(0.0F);
      sendActionFeedback(goal_handle, 2U, 0.05F, "Requesting current-control mode");
      if (!commandModeAndWait(0U, false, 0.0F, goal_handle, 2U, reason)) {
        throw reason;
      }

      sendActionFeedback(goal_handle, 3U, 0.1F, "Constant-current acceleration started");
      const auto acceleration_start = Clock::now();
      bool acceleration_complete = false;
      while (rclcpp::ok()) {
        reason = detectInterrupt(goal_handle);
        if (reason != Interrupt::NONE) {
          throw reason;
        }
        if (Clock::now() - acceleration_start >=
          std::chrono::duration<double>(acceleration_timeout_sec_))
        {
          throw Interrupt::ACCELERATION_TIMEOUT;
        }

        const auto feedback = getFeedbackSnapshot();
        if (feedback.mode != 0U) {
          throw Interrupt::MODE_MISMATCH;
        }
        const double moved_mm = std::abs(
          static_cast<double>(feedback.position) - static_cast<double>(start_position));
        publishCurrent(static_cast<float>(current_for_this_action));
        const float progress = static_cast<float>(
          std::min(0.65, 0.1 + 0.55 * moved_mm / max_acceleration_distance_mm_));
        sendActionFeedback(
          goal_handle, 3U, progress,
          "Accelerating: delta position=" + std::to_string(moved_mm) + " mm");
        if (moved_mm >= max_acceleration_distance_mm_) {
          acceleration_complete = true;
          break;
        }
        std::this_thread::sleep_for(control_period_);
      }
      if (!acceleration_complete) {
        throw Interrupt::SHUTDOWN;
      }

      // Normal path: zero current, then transition to brake mode.
      publishCurrent(0.0F);
      sendActionFeedback(goal_handle, 4U, 0.68F, "Switching to brake mode");
      if (!commandModeAndWait(3U, false, 0.0F, goal_handle, 4U, reason)) {
        throw reason;
      }
      if (!waitForBrakeVelocity(goal_handle, 4U, reason)) {
        throw reason;
      }

      // After braking, select the smallest interval multiple that is >= the
      // latest position feedback. The selected target remains fixed for this Action.
      const auto return_start_feedback = getFeedbackSnapshot();
      if (!feedbackIsFresh(return_start_feedback)) {
        throw Interrupt::FEEDBACK_TIMEOUT;
      }

      const double current_position_mm =
        static_cast<double>(return_start_feedback.position);
      double target_multiple = std::ceil(
        current_position_mm / origin_return_interval_mm_);
      double target_position_mm = target_multiple * origin_return_interval_mm_;

      if (!std::isfinite(target_multiple) || !std::isfinite(target_position_mm)) {
        throw Interrupt::INTERNAL_ERROR;
      }

      // Guard against floating-point rounding producing a target just below
      // the current position, including after conversion to the float CAN payload.
      if (target_position_mm < current_position_mm) {
        target_multiple += 1.0;
        target_position_mm = target_multiple * origin_return_interval_mm_;
      }
      if (!std::isfinite(target_position_mm) ||
        target_position_mm > static_cast<double>(std::numeric_limits<float>::max()) ||
        target_position_mm < -static_cast<double>(std::numeric_limits<float>::max()))
      {
        throw Interrupt::INTERNAL_ERROR;
      }

      float origin_target = static_cast<float>(target_position_mm);
      if (static_cast<double>(origin_target) < current_position_mm) {
        target_multiple += 1.0;
        target_position_mm = target_multiple * origin_return_interval_mm_;
        if (!std::isfinite(target_position_mm) ||
          target_position_mm > static_cast<double>(std::numeric_limits<float>::max()) ||
          target_position_mm < -static_cast<double>(std::numeric_limits<float>::max()))
        {
          throw Interrupt::INTERNAL_ERROR;
        }
        origin_target = static_cast<float>(target_position_mm);
      }
      if (!std::isfinite(origin_target) ||
        static_cast<double>(origin_target) < current_position_mm)
      {
        throw Interrupt::INTERNAL_ERROR;
      }

      RCLCPP_INFO(
        get_logger(),
        "Origin return target selected: current=%.3f mm, interval=%.3f mm, target=%.3f mm",
        current_position_mm, origin_return_interval_mm_, origin_target);
      sendActionFeedback(
        goal_handle, 5U, 0.8F,
        "Returning to the next interval multiple: " + std::to_string(origin_target) + " mm");

      publishPosition(origin_target);
      if (!waitWithSafetyChecks(
          static_cast<double>(control_period_ms_) / 1000.0,
          5U, goal_handle, reason, false, 3))
      {
        throw reason;
      }
      if (!commandModeAndWait(2U, true, origin_target, goal_handle, 5U, reason)) {
        throw reason;
      }
      if (!waitForPositionTarget(
          goal_handle, origin_target, "Holding calculated interval-multiple position", reason))
      {
        throw reason;
      }

      reason = detectInterrupt(goal_handle);
      if (reason != Interrupt::NONE) {
        throw reason;
      }
      result->success = true;
      result->final_state = 5U;  // Completed; position mode holds the calculated target.
      result->error_code = 0U;
      result->failed_actuator.clear();
      result->reason =
        "Completed; calculated interval-multiple position reached and held without origin reset";
      goal_handle->succeed(result);
      action_succeeded = true;
      RCLCPP_INFO(
        get_logger(),
        "Action completed; holding %.3f mm in position mode (origin unchanged)",
        origin_target);
    } catch (Interrupt caught) {
      reason = caught;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Unexpected exception in Action: %s", e.what());
      reason = Interrupt::INTERNAL_ERROR;
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Unknown exception in Action");
      reason = Interrupt::INTERNAL_ERROR;
    }

    if (reason != Interrupt::NONE) {
      RCLCPP_ERROR(
        get_logger(), "Stopping Action: %s; switching to brake mode without position return",
        interruptText(reason).c_str());
      commandBrakeImmediately();

      result->success = false;
      result->final_state = (reason == Interrupt::STOP_TOPIC) ? 6U :
        ((reason == Interrupt::ACTION_CANCEL) ? 8U : 7U);
      result->error_code = errorCode(reason);
      result->failed_actuator = "belt_motor";
      result->reason = interruptText(reason);

      if (reason == Interrupt::ACTION_CANCEL && goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
    }

    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      // Decide the terminal state before clearing active_. A stop/cancel/fault
      // arriving just before this lock must not be erased and followed by READY.
      if (emergency_stop_active_.load() || emergency_stop_event_.load() ||
        feedback_fault_.load())
      {
        operation_state_ = OperationState::ERROR;
      } else if (reason == Interrupt::STOP_TOPIC ||
        reason == Interrupt::ACTION_CANCEL ||
        stop_requested_.load() || cancel_requested_.load())
      {
        operation_state_ = OperationState::CANCELLED;
      } else if (action_succeeded) {
        operation_state_ = OperationState::READY;
      } else {
        operation_state_ = OperationState::ERROR;
      }
      active_.store(false);
    }
  }

  static uint16_t errorCode(Interrupt reason)
  {
    switch (reason) {
      case Interrupt::STOP_TOPIC: return 1U;
      case Interrupt::ACTION_CANCEL: return 2U;
      case Interrupt::FEEDBACK_MALFORMED: return 3U;
      case Interrupt::FEEDBACK_TIMEOUT: return 4U;
      case Interrupt::MODE_TIMEOUT: return 5U;
      case Interrupt::ACCELERATION_TIMEOUT: return 6U;
      case Interrupt::POSITION_RETURN_TIMEOUT: return 7U;
      case Interrupt::ORIGIN_RESET_TIMEOUT: return 12U;
      case Interrupt::BRAKE_TIMEOUT: return 13U;
      case Interrupt::SHUTDOWN: return 8U;
      case Interrupt::INTERNAL_ERROR: return 9U;
      case Interrupt::EMERGENCY_STOP: return 10U;
      case Interrupt::MODE_MISMATCH: return 11U;
      case Interrupt::NONE: return 0U;
    }
    return 9U;
  }

  static std::string interruptText(Interrupt reason)
  {
    switch (reason) {
      case Interrupt::STOP_TOPIC: return "Dedicated stop topic received";
      case Interrupt::EMERGENCY_STOP: return "Emergency-stop input asserted";
      case Interrupt::ACTION_CANCEL: return "Action cancellation requested";
      case Interrupt::FEEDBACK_MALFORMED: return "Malformed or non-finite CAN feedback";
      case Interrupt::FEEDBACK_TIMEOUT: return "CAN position/control-mode/velocity feedback timed out";
      case Interrupt::MODE_TIMEOUT: return "Control-mode confirmation timed out";
      case Interrupt::MODE_MISMATCH: return "Unexpected control-mode feedback during active phase";
      case Interrupt::ACCELERATION_TIMEOUT: return "Maximum acceleration distance was not reached in time";
      case Interrupt::POSITION_RETURN_TIMEOUT: return "Position target / origin hold timed out";
      case Interrupt::ORIGIN_RESET_TIMEOUT: return "Origin reset was not confirmed by fresh 0 mm position feedback";
      case Interrupt::BRAKE_TIMEOUT: return "Motor velocity did not reach the stop threshold before brake timeout";
      case Interrupt::SHUTDOWN: return "ROS shutdown requested";
      case Interrupt::INTERNAL_ERROR: return "Internal controller error";
      case Interrupt::NONE: return "No error";
    }
    return "Unknown error";
  }

  double shoot_current_a_{0.0};
  double min_shoot_current_a_{0.1};
  double max_shoot_current_a_{80.0};
  double max_acceleration_distance_mm_{585.0};
  double brake_wait_sec_{0.3};
  double brake_velocity_threshold_rad_s_{0.2};
  double brake_velocity_stable_time_sec_{0.1};
  double brake_timeout_sec_{3.0};
  double velocity_feedback_timeout_sec_{0.2};
  int control_period_ms_{10};
  double feedback_timeout_sec_{0.2};
  double startup_feedback_timeout_sec_{1.0};
  double mode_confirmation_timeout_sec_{0.5};
  double acceleration_timeout_sec_{1.5};
  double position_tolerance_mm_{1.415};
  double position_stable_time_sec_{0.1};
  double position_return_timeout_sec_{5.0};
  double origin_return_interval_mm_{1125.0};
  double origin_reset_timeout_sec_{0.5};
  double origin_reset_retry_interval_sec_{0.1};
  double active_shoot_current_a_{0.0};
  double manual_fixed_bucket_current_a_{1.0};
  double manual_bucket_1_current_a_{1.5};
  double manual_bucket_2_current_a_{0.5};
  double manual_bucket_3_current_a_{1.0};
  double manual_move_current_a_{1.0};
  double manual_current_step_a_{1.0};
  double manual_min_current_a_{0.1};
  double manual_max_current_a_{10.0};

  std::string command_topic_;
  std::string feedback_position_topic_;
  std::string feedback_velocity_topic_;
  std::string feedback_mode_topic_;
  std::string feedback_fault_topic_;
  std::string stop_topic_;
  std::string emergency_stop_topic_;
  std::string action_name_;
  std::string manual_current_service_name_;
  std::string reset_origin_service_name_;
  std::string manual_current_state_topic_;
  std::string operation_state_topic_;
  std::chrono::milliseconds control_period_{10};

  rclcpp::CallbackGroup::SharedPtr feedback_callback_group_;
  rclcpp::CallbackGroup::SharedPtr safety_callback_group_;
  rclcpp::CallbackGroup::SharedPtr origin_reset_callback_group_;
  rclcpp::Publisher<shoot_belt::msg::ShootBeltCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr manual_current_state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr operation_state_pub_;
  rclcpp::TimerBase::SharedPtr operation_state_timer_;
  rclcpp::Service<shoot_belt::srv::ManualCurrent>::SharedPtr manual_current_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_origin_service_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fault_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;

  std::mutex manual_current_mutex_;
  std::mutex goal_mutex_;
  OperationState operation_state_{OperationState::RESETTING};
  std::mutex worker_mutex_;
  std::thread worker_thread_;
  std::thread initialization_thread_;
  std::atomic<bool> initialization_complete_{false};
  std::atomic<bool> origin_initialization_active_{false};
  std::atomic<bool> startup_abort_requested_{false};
  std::atomic<bool> active_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> shutdown_requested_{false};
  std::atomic<bool> feedback_fault_{false};
  std::atomic<bool> emergency_stop_active_{false};
  std::atomic<bool> emergency_stop_event_{false};

  std::mutex feedback_mutex_;
  std::mutex command_publish_mutex_;
  FeedbackSnapshot feedback_;
  std::mutex fault_mutex_;
  std::string feedback_fault_text_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ShootBeltController>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
