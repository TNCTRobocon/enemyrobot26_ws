#include <algorithm>
#include <array>
#include <atomic>
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
#include <thread>
#include <utility>
#include <vector>

#include "can_socketcan_common/can_receiver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sequence_interfaces/action/sequence.hpp"
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
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");
    can_error_log_path_ = declare_parameter<std::string>(
      "can_error_log_path", "/tmp/shoot_belt_measurement_can_rx_error.log");
    can_error_log_interval_sec_ =
      declare_parameter<double>("can_error_log_interval_sec", 1.0);
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

    can_receiver_ = std::make_unique<can_socketcan_common::CanReceiver>(
      can_interface_, can_error_log_path_, can_error_log_interval_sec_);
    if (!can_receiver_->is_open()) {
      throw std::runtime_error(
        "Could not open SocketCAN receiver on interface " + can_interface_);
    }

    action_client_ = rclcpp_action::create_client<Action>(this, action_name_);

    start_service_ = create_service<std_srvs::srv::Trigger>(
      start_service_name_,
      std::bind(
        &ShootBeltMeasurement::onStartRequest, this,
        std::placeholders::_1, std::placeholders::_2));

    action_server_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ShootBeltMeasurement::pollForActionServer, this));
    action_server_timer_->cancel();

    can_receiver_running_.store(true);
    can_receiver_thread_ = std::thread(
      &ShootBeltMeasurement::canReceiveLoop, this);

    RCLCPP_INFO(
      get_logger(),
      "Measurement node ready. Call %s to send one /shoot_belt/execute Goal and record it.",
      start_service_name_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Watching SocketCAN interface=%s (current ID 0x100, position ID 0x220, velocity ID 0x210)",
      can_interface_.c_str());
  }

  ~ShootBeltMeasurement() override
  {
    can_receiver_running_.store(false);
    if (can_receiver_thread_.joinable()) {
      can_receiver_thread_.join();
    }
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
    bool have_position_change{false};
    bool have_position_confirmation{false};
    bool goal_accepted{false};
    bool have_position_at_action_start{false};
    bool have_velocity_at_action_start{false};
    bool have_position_at_current_can{false};

    SystemClock::time_point action_start_system{};
    SteadyTime action_start{};
    SteadyTime current_can_time{};
    SteadyTime position_change_first_time{};
    SteadyTime position_change_confirmation_time{};

    double position_at_action_start_mm{0.0};
    double velocity_at_action_start_rad_s{0.0};
    double position_at_current_can_mm{0.0};
    double current_a{0.0};
    double action_to_current_can_socketcan_receive_ms{0.0};
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
    if (action_name_.empty() || can_interface_.empty() ||
      can_error_log_path_.empty() || start_service_name_.empty() ||
      output_directory_.empty())
    {
      throw std::invalid_argument("Action, CAN interface, service, and output paths must not be empty");
    }
    if (!std::isfinite(action_server_wait_timeout_sec_) ||
      action_server_wait_timeout_sec_ <= 0.0 ||
      !std::isfinite(initial_feedback_max_age_sec_) ||
      initial_feedback_max_age_sec_ <= 0.0 ||
      !std::isfinite(current_zero_epsilon_a_) || current_zero_epsilon_a_ < 0.0 ||
      !std::isfinite(position_change_threshold_mm_) ||
      position_change_threshold_mm_ <= 0.0 ||
      !std::isfinite(can_error_log_interval_sec_) ||
      can_error_log_interval_sec_ < 0.0 ||
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
      action_to_current_can_socketcan_receive_ms_ = 0.0;
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

  void canReceiveLoop()
  {
    while (can_receiver_running_.load()) {
      can_socketcan_common::ReceivedCanFrame frame;
      const auto status = can_receiver_->receive(frame, 100);
      if (status == can_socketcan_common::ReceiveStatus::Timeout) {
        continue;
      }
      if (status == can_socketcan_common::ReceiveStatus::Error) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }

      if (frame.extended || frame.remote_request) {
        continue;
      }

      if (frame.can_id != 0x100U && frame.can_id != 0x220U &&
        frame.can_id != 0x210U)
      {
        continue;
      }
      if (frame.dlc != 4U) {
        RCLCPP_WARN(
          get_logger(), "Ignoring CAN ID 0x%03X with DLC=%u (expected 4)",
          frame.can_id, static_cast<unsigned>(frame.dlc));
        continue;
      }

      const float value = decodeFloat32LE(frame.data);
      if (!std::isfinite(value)) {
        RCLCPP_WARN(
          get_logger(), "Ignoring non-finite float32 value on CAN ID 0x%03X",
          frame.can_id);
        continue;
      }

      // Timestamp immediately after the SocketCAN receive call returns.
      // All three signals therefore use the same steady-clock domain.
      const auto receive_time = SteadyClock::now();
      if (frame.can_id == 0x100U) {
        onCurrentCan(value, receive_time);
      } else if (frame.can_id == 0x220U) {
        onPosition(value, receive_time);
      } else if (frame.can_id == 0x210U) {
        onVelocity(value, receive_time);
      }
    }
  }

  void onCurrentCan(float current_a, const SteadyTime & receive_time)
  {
    if (std::abs(static_cast<double>(current_a)) <= current_zero_epsilon_a_) {
      return;
    }

    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!measurement_active_ || have_current_can_) {
      return;
    }

    have_current_can_ = true;
    current_can_time_ = receive_time;
    current_can_a_ = static_cast<double>(current_a);
    action_to_current_can_socketcan_receive_ms_ =
      elapsedMilliseconds(action_start_, receive_time);

    // The same SocketCAN receive thread processes position and current frames.
    // Select the latest position sample observed no later than this current frame.
    const auto baseline = std::find_if(
      position_history_.rbegin(), position_history_.rend(),
      [&receive_time](const TimedValue & sample) {
        return sample.time <= receive_time;
      });
    if (baseline != position_history_.rend()) {
      const double baseline_age_sec = std::chrono::duration<double>(
        receive_time - baseline->time).count();
      if (baseline_age_sec <= initial_feedback_max_age_sec_) {
        have_position_at_current_can_ = true;
        position_at_current_can_mm_ = static_cast<double>(baseline->value);
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "First non-zero current CAN received: %.4f A, action-to-SocketCAN-RX=%.3f ms",
      current_can_a_,
      action_to_current_can_socketcan_receive_ms_);
  }

  void onPosition(float position_mm, const SteadyTime & now)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_position_.valid = true;
    latest_position_.value = position_mm;
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
      static_cast<double>(position_mm) - position_at_action_start_mm_);
    max_abs_position_delta_from_action_start_mm_ = std::max(
      max_abs_position_delta_from_action_start_mm_, action_displacement);

    if (!have_current_can_ || !have_position_at_current_can_ ||
      have_position_change_)
    {
      return;
    }

    const double current_can_displacement = std::abs(
      static_cast<double>(position_mm) - position_at_current_can_mm_);

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

  void onVelocity(float velocity_rad_s, const SteadyTime & now)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_velocity_.valid = true;
    latest_velocity_.value = velocity_rad_s;
    latest_velocity_.time = now;

    if (!measurement_active_) {
      return;
    }

    max_abs_velocity_rad_s_ = std::max(
      max_abs_velocity_rad_s_, std::abs(static_cast<double>(velocity_rad_s)));
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
      snapshot.have_position_change = have_position_change_;
      snapshot.have_position_confirmation = have_position_confirmation_;
      snapshot.goal_accepted = goal_accepted_;
      snapshot.have_position_at_action_start = have_position_at_action_start_;
      snapshot.have_velocity_at_action_start = have_velocity_at_action_start_;
      snapshot.have_position_at_current_can = have_position_at_current_can_;
      snapshot.action_start_system = action_start_system_;
      snapshot.action_start = action_start_;
      snapshot.current_can_time = current_can_time_;
      snapshot.position_change_first_time = position_change_first_time_;
      snapshot.position_change_confirmation_time = position_change_confirmation_time_;
      snapshot.position_at_action_start_mm = position_at_action_start_mm_;
      snapshot.velocity_at_action_start_rad_s = velocity_at_action_start_rad_s_;
      snapshot.position_at_current_can_mm = position_at_current_can_mm_;
      snapshot.current_a = current_can_a_;
      snapshot.action_to_current_can_socketcan_receive_ms =
        action_to_current_can_socketcan_receive_ms_;
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
              << "action_to_current_can_socketcan_receive_ms,"
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
      writeOptional(summary, snapshot.have_current_can,
        snapshot.action_to_current_can_socketcan_receive_ms);
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
  std::string can_interface_;
  std::string can_error_log_path_;
  std::string start_service_name_;
  std::string output_directory_;
  double can_error_log_interval_sec_{1.0};

  double action_server_wait_timeout_sec_{10.0};
  double initial_feedback_max_age_sec_{0.2};
  double current_zero_epsilon_a_{0.001};
  double position_change_threshold_mm_{1.0};
  int position_change_confirmation_samples_{3};

  rclcpp_action::Client<Action>::SharedPtr action_client_;
  std::unique_ptr<can_socketcan_common::CanReceiver> can_receiver_;
  std::atomic<bool> can_receiver_running_{false};
  std::thread can_receiver_thread_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::TimerBase::SharedPtr action_server_timer_;

  std::mutex data_mutex_;
  bool waiting_for_action_server_{false};
  bool measurement_active_{false};
  bool goal_accepted_{false};
  bool have_position_at_action_start_{false};
  bool have_velocity_at_action_start_{false};
  bool have_current_can_{false};
  bool have_position_at_current_can_{false};
  bool have_position_change_{false};
  bool have_position_confirmation_{false};

  SteadyTime action_server_deadline_{};
  SteadyTime action_start_{};
  SystemClock::time_point action_start_system_{};
  SteadyTime current_can_time_{};
  SteadyTime position_change_first_time_{};
  SteadyTime position_change_confirmation_time_{};

  TimedValue latest_position_;
  TimedValue latest_velocity_;
  std::deque<TimedValue> position_history_;
  double position_at_action_start_mm_{0.0};
  double velocity_at_action_start_rad_s_{0.0};
  double position_at_current_can_mm_{0.0};
  double current_can_a_{0.0};
  double action_to_current_can_socketcan_receive_ms_{0.0};
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
