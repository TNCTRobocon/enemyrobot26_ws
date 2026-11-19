#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sequence_interfaces/action/sequence.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"

class ShootBeltMeasurement final : public rclcpp::Node
{
public:
  using Action = sequence_interfaces::action::Sequence;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  using SteadyClock = std::chrono::steady_clock;
  using SteadyTime = SteadyClock::time_point;
  using SystemClock = std::chrono::system_clock;

  ShootBeltMeasurement()
  : Node("shoot_belt_measurement")
  {
    action_name_ = declare_parameter<std::string>("action_name", "/shoot_belt/execute");
    can_tx_topic_ = declare_parameter<std::string>("can_tx_topic", "/can_tx");
    position_topic_ = declare_parameter<std::string>(
      "position_topic", "/shoot_belt/feedback/position");
    velocity_topic_ = declare_parameter<std::string>(
      "velocity_topic", "/shoot_belt/feedback/velocity");
    start_service_name_ = declare_parameter<std::string>(
      "start_service_name", "/shoot_belt_measurement/start");
    output_directory_ = declare_parameter<std::string>(
      "output_directory", "~/.ros/shoot_belt_measurement");

    action_server_wait_timeout_sec_ =
      declare_parameter<double>("action_server_wait_timeout_sec", 10.0);
    initial_feedback_max_age_sec_ =
      declare_parameter<double>("initial_feedback_max_age_sec", 0.2);
    current_zero_epsilon_a_ =
      declare_parameter<double>("current_zero_epsilon_a", 0.001);
    position_change_threshold_mm_ =
      declare_parameter<double>("position_change_threshold_mm", 1.0);
    position_change_confirmation_samples_ =
      declare_parameter<int>("position_change_confirmation_samples", 3);

    validateParameters();
    action_client_ = rclcpp_action::create_client<Action>(this, action_name_);

    can_tx_sub_ = create_subscription<can_msgs::msg::Frame>(
      can_tx_topic_, rclcpp::QoS(200),
      std::bind(&ShootBeltMeasurement::onCanTx, this, std::placeholders::_1));
    position_sub_ = create_subscription<std_msgs::msg::Float32>(
      position_topic_, rclcpp::QoS(200),
      std::bind(&ShootBeltMeasurement::onPosition, this, std::placeholders::_1));
    velocity_sub_ = create_subscription<std_msgs::msg::Float32>(
      velocity_topic_, rclcpp::QoS(200),
      std::bind(&ShootBeltMeasurement::onVelocity, this, std::placeholders::_1));

    start_service_ = create_service<std_srvs::srv::Trigger>(
      start_service_name_,
      std::bind(
        &ShootBeltMeasurement::onStartRequest, this,
        std::placeholders::_1, std::placeholders::_2));

    action_server_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ShootBeltMeasurement::pollForActionServer, this));
    action_server_timer_->cancel();

    RCLCPP_INFO(
      get_logger(),
      "Measurement node ready. Call %s to send one /shoot_belt/execute Goal and record it.",
      start_service_name_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Watching CAN TX=%s (current ID 0x100), position=%s, velocity=%s",
      can_tx_topic_.c_str(), position_topic_.c_str(), velocity_topic_.c_str());
  }

private:
  struct TimedValue
  {
    bool valid{false};
    float value{0.0F};
    SteadyTime time{};
  };

  struct SeriesRow
  {
    double elapsed_since_action_ms{0.0};
    std::string updated_signal;
    bool have_position{false};
    double position_elapsed_ms{0.0};
    double position_mm{0.0};
    double position_age_ms{0.0};
    bool have_velocity{false};
    double velocity_elapsed_ms{0.0};
    double velocity_rad_s{0.0};
    double velocity_age_ms{0.0};
  };

  struct MeasurementSnapshot
  {
    bool have_current_can{false};
    bool have_current_can_publish_stamp{false};
    bool have_position_change{false};
    bool have_position_confirmation{false};
    bool goal_accepted{false};
    bool have_position_at_action_start{false};
    bool have_velocity_at_action_start{false};
    bool have_position_at_current_can{false};

    SystemClock::time_point action_start_system{};
    SteadyTime action_start{};
    SteadyTime current_can_time{};
    SteadyTime current_can_receive_time{};
    SteadyTime position_change_first_time{};
    SteadyTime position_change_confirmation_time{};

    double position_at_action_start_mm{0.0};
    double velocity_at_action_start_rad_s{0.0};
    double position_at_current_can_mm{0.0};
    double current_a{0.0};
    double action_to_current_can_publish_ms{0.0};
    double max_abs_velocity_rad_s{0.0};
    double max_abs_position_delta_from_action_start_mm{0.0};

    int last_action_phase{-1};
    double last_action_progress{0.0};
    std::string action_result_code{"UNKNOWN"};
    std::string action_result_reason;
    std::vector<SeriesRow> rows;
  };

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

  static double elapsedMilliseconds(const SteadyTime & start, const SteadyTime & end)
  {
    return std::chrono::duration<double, std::milli>(end - start).count();
  }

  static std::string csvEscape(const std::string & value)
  {
    std::string escaped = "\"";
    for (const char character : value) {
      if (character == '"') {
        escaped += "\"\"";
      } else {
        escaped += character;
      }
    }
    escaped += "\"";
    return escaped;
  }

  static std::string formatSystemTime(const SystemClock::time_point & point)
  {
    const auto time = SystemClock::to_time_t(point);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
      point.time_since_epoch()).count() % 1000;
    std::tm local_time{};
    localtime_r(&time, &local_time);

    std::ostringstream stream;
    stream << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S")
           << '.' << std::setfill('0') << std::setw(3) << milliseconds;
    return stream.str();
  }

  static void writeOptional(std::ostream & stream, bool valid, double value)
  {
    if (valid) {
      stream << value;
    }
  }

  void validateParameters() const
  {
    if (action_name_.empty() || can_tx_topic_.empty() ||
      position_topic_.empty() || velocity_topic_.empty() ||
      start_service_name_.empty() || output_directory_.empty())
    {
      throw std::invalid_argument("Topic, action, service, and output paths must not be empty");
    }
    if (!std::isfinite(action_server_wait_timeout_sec_) ||
      action_server_wait_timeout_sec_ <= 0.0 ||
      !std::isfinite(initial_feedback_max_age_sec_) ||
      initial_feedback_max_age_sec_ <= 0.0 ||
      !std::isfinite(current_zero_epsilon_a_) || current_zero_epsilon_a_ < 0.0 ||
      !std::isfinite(position_change_threshold_mm_) ||
      position_change_threshold_mm_ <= 0.0 ||
      position_change_confirmation_samples_ < 1)
    {
      throw std::invalid_argument("One or more measurement parameters are invalid");
    }
  }

  void onStartRequest(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (measurement_active_ || waiting_for_action_server_) {
        response->success = false;
        response->message = "A measurement is already active or waiting to start";
        return;
      }
      waiting_for_action_server_ = true;
      action_server_deadline_ = SteadyClock::now() +
        std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(action_server_wait_timeout_sec_));
    }

    response->success = true;
    response->message =
      "Measurement request accepted. Waiting for the Action server and fresh position feedback.";
    action_server_timer_->reset();
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void pollForActionServer()
  {
    bool waiting = false;
    bool timed_out = false;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      waiting = waiting_for_action_server_;
      timed_out = waiting && SteadyClock::now() >= action_server_deadline_;
    }
    if (!waiting) {
      action_server_timer_->cancel();
      return;
    }

    if (action_client_->action_server_is_ready()) {
      const auto now = SteadyClock::now();
      bool position_is_fresh = false;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (latest_position_.valid) {
          const double age = std::chrono::duration<double>(
            now - latest_position_.time).count();
          position_is_fresh = age <= initial_feedback_max_age_sec_;
        }
      }

      if (position_is_fresh) {
        sendMeasurementGoal();
        return;
      }
    }

    if (timed_out) {
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        waiting_for_action_server_ = false;
      }
      action_server_timer_->cancel();
      RCLCPP_ERROR(
        get_logger(),
        "Measurement start timed out: Action server was unavailable or fresh position feedback "
        "was not received within %.3f s",
        action_server_wait_timeout_sec_);
    }
  }

  void sendMeasurementGoal()
  {
    Action::Goal goal;
    goal.operation = Action::Goal::OPERATION_FULL_CYCLE;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!waiting_for_action_server_ || measurement_active_ ||
        !latest_position_.valid)
      {
        waiting_for_action_server_ = false;
        action_server_timer_->cancel();
        RCLCPP_ERROR(get_logger(), "Cannot start measurement: start state changed unexpectedly");
        return;
      }

      const auto now = SteadyClock::now();
      const double position_age = std::chrono::duration<double>(
        now - latest_position_.time).count();
      if (position_age > initial_feedback_max_age_sec_) {
        // Keep polling until a fresh sample arrives or the start timeout expires.
        return;
      }

      rows_.clear();
      have_current_can_ = false;
      have_current_can_publish_stamp_ = false;
      action_to_current_can_publish_ms_ = 0.0;
      have_position_change_ = false;
      have_position_confirmation_ = false;
      have_position_at_current_can_ = false;
      have_velocity_at_action_start_ = false;
      velocity_at_action_start_rad_s_ = 0.0;
      goal_accepted_ = false;
      consecutive_position_change_samples_ = 0;
      last_action_phase_ = -1;
      last_action_progress_ = 0.0;
      max_abs_velocity_rad_s_ = 0.0;
      max_abs_position_delta_from_action_start_mm_ = 0.0;
      action_result_code_ = "PENDING";
      action_result_reason_.clear();

      position_at_action_start_mm_ = latest_position_.value;
      have_position_at_action_start_ = true;
      measurement_active_ = true;
      waiting_for_action_server_ = false;
    }
    action_server_timer_->cancel();

    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback =
      [this](const GoalHandle::SharedPtr goal_handle)
      {
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "shoot_belt rejected the measurement Action Goal");
          finishMeasurement("REJECTED", "Action Goal was rejected by shoot_belt");
          return;
        }

        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          goal_accepted_ = true;
        }
        RCLCPP_INFO(get_logger(), "shoot_belt accepted the measurement Action Goal");
      };

    options.feedback_callback =
      [this](
      GoalHandle::SharedPtr,
      const std::shared_ptr<const Action::Feedback> feedback)
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!measurement_active_) {
          return;
        }
        last_action_phase_ = static_cast<int>(feedback->phase);
        last_action_progress_ = static_cast<double>(feedback->progress);
      };

    options.result_callback =
      [this](const GoalHandle::WrappedResult & result)
      {
        std::string result_code;
        std::string reason;
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            result_code = "SUCCEEDED";
            break;
          case rclcpp_action::ResultCode::CANCELED:
            result_code = "CANCELED";
            break;
          case rclcpp_action::ResultCode::ABORTED:
            result_code = "ABORTED";
            break;
          default:
            result_code = "UNKNOWN";
            break;
        }

        if (result.result) {
          reason = result.result->reason;
          if (!result.result->success && reason.empty()) {
            reason = "Action returned success=false";
          }
        } else {
          reason = "Action result message was empty";
        }
        finishMeasurement(result_code, reason);
      };

    try {
      RCLCPP_INFO(
        get_logger(),
        "Sending /shoot_belt/execute Goal (operation=%u)",
        static_cast<unsigned>(goal.operation));
      {
        // Capture the baseline and clocks immediately before invoking the Action
        // Client send API. No logging or other work is performed between this
        // timestamp and async_send_goal().
        std::lock_guard<std::mutex> lock(data_mutex_);
        action_start_ = SteadyClock::now();
        action_start_system_ = SystemClock::now();
        action_start_ros_time_ns_ = get_clock()->now().nanoseconds();
        position_at_action_start_mm_ = latest_position_.value;
        have_velocity_at_action_start_ = latest_velocity_.valid;
        if (have_velocity_at_action_start_) {
          velocity_at_action_start_rad_s_ =
            static_cast<double>(latest_velocity_.value);
          max_abs_velocity_rad_s_ = std::abs(velocity_at_action_start_rad_s_);
        }
        appendSeriesRowLocked(action_start_, "action_start");
      }
      (void)action_client_->async_send_goal(goal, options);
    } catch (const std::exception & exception) {
      finishMeasurement(
        "SEND_FAILED", std::string("Failed to send Action Goal: ") + exception.what());
    }
  }

  void onCanTx(const can_msgs::msg::Frame::SharedPtr frame)
  {
    if (frame->is_extended || frame->is_error || frame->is_rtr ||
      frame->id != 0x100U || frame->dlc != 4U)
    {
      return;
    }

    const float current_a = decodeFloat32LE(frame->data);
    if (!std::isfinite(current_a) ||
      std::abs(static_cast<double>(current_a)) <= current_zero_epsilon_a_)
    {
      return;
    }

    const auto receive_time = SteadyClock::now();
    const auto frame_stamp_ns = rclcpp::Time(frame->header.stamp).nanoseconds();
    const auto ros_receive_time_ns = get_clock()->now().nanoseconds();
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!measurement_active_ || have_current_can_) {
      return;
    }

    have_current_can_ = true;
    current_can_receive_time_ = receive_time;
    current_can_time_ = receive_time;
    current_can_a_ = static_cast<double>(current_a);

    // The CAN TX node stamps the frame immediately before publishing it.
    // Use that timestamp for Action-to-CAN latency when both nodes share ROS time.
    if (frame_stamp_ns > 0 && action_start_ros_time_ns_ > 0) {
      const auto publish_delta_ns = frame_stamp_ns - action_start_ros_time_ns_;
      if (publish_delta_ns >= 0 && publish_delta_ns <= 60000000000LL) {
        have_current_can_publish_stamp_ = true;
        action_to_current_can_publish_ms_ =
          static_cast<double>(publish_delta_ns) / 1.0e6;
      }

      // Map the frame's ROS timestamp into this node's steady-clock domain.
      // If the clocks are not comparable, retain the local receive timestamp.
      const auto transport_age_ns = ros_receive_time_ns - frame_stamp_ns;
      if (transport_age_ns >= 0 && transport_age_ns <= 2000000000LL) {
        current_can_time_ = receive_time - std::chrono::nanoseconds(transport_age_ns);
      }
    }

    // Select the most recent position sample received no later than the CAN
    // publisher timestamp. A position callback may be dispatched before the CAN
    // callback even when its sample was published after the CAN frame.
    const auto baseline = std::find_if(
      position_history_.rbegin(), position_history_.rend(),
      [this](const TimedValue & sample) {
        return sample.time <= current_can_time_;
      });
    if (baseline != position_history_.rend()) {
      const double baseline_age_sec = std::chrono::duration<double>(
        current_can_time_ - baseline->time).count();
      if (baseline_age_sec <= initial_feedback_max_age_sec_) {
        have_position_at_current_can_ = true;
        position_at_current_can_mm_ = static_cast<double>(baseline->value);
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "First non-zero current CAN detected: %.4f A, action-to-CAN=%.3f ms",
      current_can_a_, elapsedMilliseconds(action_start_, current_can_time_));
  }

  void onPosition(const std_msgs::msg::Float32::SharedPtr message)
  {
    if (!std::isfinite(message->data)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring non-finite position feedback");
      return;
    }

    const auto now = SteadyClock::now();
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_position_.valid = true;
    latest_position_.value = message->data;
    latest_position_.time = now;
    position_history_.push_back(latest_position_);
    while (position_history_.size() > 1000U) {
      position_history_.pop_front();
    }
    while (!position_history_.empty() &&
      std::chrono::duration<double>(now - position_history_.front().time).count() > 2.0)
    {
      position_history_.pop_front();
    }

    if (!measurement_active_) {
      return;
    }

    appendSeriesRowLocked(now, "position");
    const double action_displacement = std::abs(
      static_cast<double>(message->data) - position_at_action_start_mm_);
    max_abs_position_delta_from_action_start_mm_ = std::max(
      max_abs_position_delta_from_action_start_mm_, action_displacement);

    if (!have_current_can_ || !have_position_at_current_can_ ||
      have_position_change_)
    {
      return;
    }

    const double current_can_displacement = std::abs(
      static_cast<double>(message->data) - position_at_current_can_mm_);

    if (current_can_displacement >= position_change_threshold_mm_) {
      if (consecutive_position_change_samples_ == 0) {
        position_change_first_time_ = now;
      }
      ++consecutive_position_change_samples_;

      if (consecutive_position_change_samples_ >=
        position_change_confirmation_samples_)
      {
        have_position_change_ = true;
        have_position_confirmation_ = true;
        position_change_confirmation_time_ = now;
        RCLCPP_INFO(
          get_logger(),
          "Position change confirmed: delta=%.4f mm, first crossing after CAN=%.3f ms "
          "(confirmation=%.3f ms)",
          current_can_displacement,
          elapsedMilliseconds(current_can_time_, position_change_first_time_),
          elapsedMilliseconds(current_can_time_, position_change_confirmation_time_));
      }
    } else {
      consecutive_position_change_samples_ = 0;
    }
  }

  void onVelocity(const std_msgs::msg::Float32::SharedPtr message)
  {
    if (!std::isfinite(message->data)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring non-finite velocity feedback");
      return;
    }

    const auto now = SteadyClock::now();
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_velocity_.valid = true;
    latest_velocity_.value = message->data;
    latest_velocity_.time = now;

    if (!measurement_active_) {
      return;
    }

    max_abs_velocity_rad_s_ = std::max(
      max_abs_velocity_rad_s_, std::abs(static_cast<double>(message->data)));
    appendSeriesRowLocked(now, "velocity");
  }

  void appendSeriesRowLocked(const SteadyTime & now, const std::string & source)
  {
    SeriesRow row;
    row.elapsed_since_action_ms = elapsedMilliseconds(action_start_, now);
    row.updated_signal = source;

    if (latest_position_.valid) {
      row.have_position = true;
      row.position_elapsed_ms = elapsedMilliseconds(action_start_, latest_position_.time);
      row.position_mm = static_cast<double>(latest_position_.value);
      row.position_age_ms = elapsedMilliseconds(latest_position_.time, now);
    }

    if (latest_velocity_.valid) {
      row.have_velocity = true;
      row.velocity_elapsed_ms = elapsedMilliseconds(action_start_, latest_velocity_.time);
      row.velocity_rad_s = static_cast<double>(latest_velocity_.value);
      row.velocity_age_ms = elapsedMilliseconds(latest_velocity_.time, now);
    }

    rows_.push_back(std::move(row));
  }

  void finishMeasurement(const std::string & result_code, const std::string & reason)
  {
    MeasurementSnapshot snapshot;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!measurement_active_) {
        return;
      }

      measurement_active_ = false;
      action_result_code_ = result_code;
      action_result_reason_ = reason;

      snapshot.have_current_can = have_current_can_;
      snapshot.have_current_can_publish_stamp = have_current_can_publish_stamp_;
      snapshot.have_position_change = have_position_change_;
      snapshot.have_position_confirmation = have_position_confirmation_;
      snapshot.goal_accepted = goal_accepted_;
      snapshot.have_position_at_action_start = have_position_at_action_start_;
      snapshot.have_velocity_at_action_start = have_velocity_at_action_start_;
      snapshot.have_position_at_current_can = have_position_at_current_can_;
      snapshot.action_start_system = action_start_system_;
      snapshot.action_start = action_start_;
      snapshot.current_can_time = current_can_time_;
      snapshot.current_can_receive_time = current_can_receive_time_;
      snapshot.position_change_first_time = position_change_first_time_;
      snapshot.position_change_confirmation_time = position_change_confirmation_time_;
      snapshot.position_at_action_start_mm = position_at_action_start_mm_;
      snapshot.velocity_at_action_start_rad_s = velocity_at_action_start_rad_s_;
      snapshot.position_at_current_can_mm = position_at_current_can_mm_;
      snapshot.current_a = current_can_a_;
      snapshot.action_to_current_can_publish_ms = action_to_current_can_publish_ms_;
      snapshot.max_abs_velocity_rad_s = max_abs_velocity_rad_s_;
      snapshot.max_abs_position_delta_from_action_start_mm =
        max_abs_position_delta_from_action_start_mm_;
      snapshot.last_action_phase = last_action_phase_;
      snapshot.last_action_progress = last_action_progress_;
      snapshot.action_result_code = action_result_code_;
      snapshot.action_result_reason = action_result_reason_;
      snapshot.rows = rows_;
    }

    writeMeasurementFiles(snapshot);
  }

  std::filesystem::path expandOutputDirectory() const
  {
    if (!output_directory_.empty() && output_directory_[0] == '~') {
      const char * home = std::getenv("HOME");
      if (home != nullptr && (output_directory_.size() == 1U ||
        output_directory_[1] == '/'))
      {
        return std::filesystem::path(home) / output_directory_.substr(
          output_directory_.size() == 1U ? 1U : 2U);
      }
    }
    return std::filesystem::path(output_directory_);
  }

  void writeMeasurementFiles(const MeasurementSnapshot & snapshot)
  {
    try {
      const auto base_directory = expandOutputDirectory();
      std::filesystem::create_directories(base_directory);

      const auto start_time = SystemClock::to_time_t(snapshot.action_start_system);
      std::tm local_time{};
      localtime_r(&start_time, &local_time);
      const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        snapshot.action_start_system.time_since_epoch()).count() % 1000;

      std::ostringstream session_name;
      session_name << std::put_time(&local_time, "%Y%m%d_%H%M%S")
                   << '_' << std::setfill('0') << std::setw(3) << milliseconds;
      auto session_directory = base_directory / session_name.str();
      for (unsigned suffix = 1U; std::filesystem::exists(session_directory); ++suffix) {
        session_directory = base_directory /
          (session_name.str() + "_" + std::to_string(suffix));
      }
      std::filesystem::create_directories(session_directory);

      const auto summary_path = session_directory / "summary.csv";
      std::ofstream summary(summary_path);
      if (!summary.is_open()) {
        throw std::runtime_error("Could not open " + summary_path.string());
      }

      const bool have_action_to_position = snapshot.have_position_change;
      const bool have_can_to_position =
        snapshot.have_current_can && snapshot.have_position_change;
      const bool have_can_to_confirmation =
        snapshot.have_current_can && snapshot.have_position_confirmation;

      summary << "action_start_local_time,goal_accepted,action_result_code,action_result_reason,"
              << "initial_position_mm,initial_velocity_rad_s,current_can_a,"
              << "action_to_current_can_publish_ms,action_to_current_can_receive_ms,"
              << "position_at_current_can_mm,position_change_threshold_mm,"
              << "action_to_position_change_ms,current_can_to_position_change_ms,"
              << "current_can_to_position_confirmation_ms,max_abs_velocity_rad_s,"
              << "max_abs_position_delta_from_action_start_mm,last_action_phase,"
              << "last_action_progress\n";
      summary << csvEscape(formatSystemTime(snapshot.action_start_system)) << ','
              << (snapshot.goal_accepted ? "true" : "false") << ','
              << csvEscape(snapshot.action_result_code) << ','
              << csvEscape(snapshot.action_result_reason) << ',';
      writeOptional(summary, snapshot.have_position_at_action_start,
        snapshot.position_at_action_start_mm);
      summary << ',';
      writeOptional(summary, snapshot.have_velocity_at_action_start,
        snapshot.velocity_at_action_start_rad_s);
      summary << ',';
      writeOptional(summary, snapshot.have_current_can, snapshot.current_a);
      summary << ',';
      writeOptional(summary, snapshot.have_current_can_publish_stamp,
        snapshot.action_to_current_can_publish_ms);
      summary << ',';
      writeOptional(summary, snapshot.have_current_can,
        elapsedMilliseconds(snapshot.action_start, snapshot.current_can_receive_time));
      summary << ',';
      writeOptional(summary, snapshot.have_position_at_current_can,
        snapshot.position_at_current_can_mm);
      summary << ',' << position_change_threshold_mm_ << ',';
      writeOptional(summary, have_action_to_position,
        elapsedMilliseconds(snapshot.action_start, snapshot.position_change_first_time));
      summary << ',';
      writeOptional(summary, have_can_to_position,
        elapsedMilliseconds(snapshot.current_can_time, snapshot.position_change_first_time));
      summary << ',';
      writeOptional(summary, have_can_to_confirmation,
        elapsedMilliseconds(snapshot.current_can_time, snapshot.position_change_confirmation_time));
      summary << ',' << snapshot.max_abs_velocity_rad_s << ','
              << snapshot.max_abs_position_delta_from_action_start_mm << ','
              << snapshot.last_action_phase << ','
              << snapshot.last_action_progress << '\n';
      summary.close();

      const auto series_path = session_directory / "timeseries.csv";
      std::ofstream series(series_path);
      if (!series.is_open()) {
        throw std::runtime_error("Could not open " + series_path.string());
      }

      series << "elapsed_since_action_start_ms,updated_signal,"
             << "position_sample_elapsed_ms,position_mm,position_age_ms,"
             << "velocity_sample_elapsed_ms,velocity_rad_s,velocity_age_ms\n";
      series << std::fixed << std::setprecision(6);
      for (const auto & row : snapshot.rows) {
        series << row.elapsed_since_action_ms << ','
               << row.updated_signal << ',';
        writeOptional(series, row.have_position, row.position_elapsed_ms);
        series << ',';
        writeOptional(series, row.have_position, row.position_mm);
        series << ',';
        writeOptional(series, row.have_position, row.position_age_ms);
        series << ',';
        writeOptional(series, row.have_velocity, row.velocity_elapsed_ms);
        series << ',';
        writeOptional(series, row.have_velocity, row.velocity_rad_s);
        series << ',';
        writeOptional(series, row.have_velocity, row.velocity_age_ms);
        series << '\n';
      }
      series.close();

      RCLCPP_INFO(
        get_logger(),
        "Measurement saved: %s (summary.csv, timeseries.csv; %zu feedback rows)",
        session_directory.string().c_str(), snapshot.rows.size());
      if (!snapshot.have_current_can) {
        RCLCPP_WARN(get_logger(), "No non-zero current CAN frame was observed");
      }
      if (!snapshot.have_position_change) {
        RCLCPP_WARN(
          get_logger(),
          "Position-change threshold (%.3f mm) was not confirmed",
          position_change_threshold_mm_);
      }
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(), "Failed to save measurement data: %s", exception.what());
    }
  }

  std::string action_name_;
  std::string can_tx_topic_;
  std::string position_topic_;
  std::string velocity_topic_;
  std::string start_service_name_;
  std::string output_directory_;

  double action_server_wait_timeout_sec_{10.0};
  double initial_feedback_max_age_sec_{0.2};
  double current_zero_epsilon_a_{0.001};
  double position_change_threshold_mm_{1.0};
  int position_change_confirmation_samples_{3};

  rclcpp_action::Client<Action>::SharedPtr action_client_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_tx_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr velocity_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::TimerBase::SharedPtr action_server_timer_;

  std::mutex data_mutex_;
  bool waiting_for_action_server_{false};
  bool measurement_active_{false};
  bool goal_accepted_{false};
  bool have_position_at_action_start_{false};
  bool have_velocity_at_action_start_{false};
  bool have_current_can_{false};
  bool have_current_can_publish_stamp_{false};
  bool have_position_at_current_can_{false};
  bool have_position_change_{false};
  bool have_position_confirmation_{false};

  SteadyTime action_server_deadline_{};
  SteadyTime action_start_{};
  SystemClock::time_point action_start_system_{};
  SteadyTime current_can_time_{};
  SteadyTime current_can_receive_time_{};
  SteadyTime position_change_first_time_{};
  SteadyTime position_change_confirmation_time_{};

  TimedValue latest_position_;
  TimedValue latest_velocity_;
  std::deque<TimedValue> position_history_;
  double position_at_action_start_mm_{0.0};
  double velocity_at_action_start_rad_s_{0.0};
  double position_at_current_can_mm_{0.0};
  double current_can_a_{0.0};
  int64_t action_start_ros_time_ns_{0};
  double action_to_current_can_publish_ms_{0.0};
  int consecutive_position_change_samples_{0};
  double max_abs_velocity_rad_s_{0.0};
  double max_abs_position_delta_from_action_start_mm_{0.0};
  int last_action_phase_{-1};
  double last_action_progress_{0.0};
  std::string action_result_code_{"UNKNOWN"};
  std::string action_result_reason_;
  std::vector<SeriesRow> rows_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ShootBeltMeasurement>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
