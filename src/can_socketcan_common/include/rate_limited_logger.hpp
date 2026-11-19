#pragma once
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>

namespace can_socketcan_common::detail {
struct State {
  bool active{false};
  std::uint64_t total{0};
  std::uint64_t suppressed{0};
  std::chrono::steady_clock::time_point first{};
  std::chrono::steady_clock::time_point last{};
};

class RateLimitedLogger {
public:
  RateLimitedLogger(std::string path, double interval_sec)
  : path_(std::move(path)), interval_(std::max(0.0, interval_sec)) {}

  void error(const std::string & key, const std::string & message) noexcept {
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      auto & s = states_[key];
      const auto now = std::chrono::steady_clock::now();
      ++s.total;
      if (!s.active) {
        s.active=true; s.first=now; s.last=now;
        write("ERROR", message, s, 0);
      } else if (std::chrono::duration<double>(now-s.last).count() >= interval_) {
        write("STILL_ERROR", message, s, s.suppressed);
        s.suppressed=0; s.last=now;
      } else {
        ++s.suppressed;
      }
    } catch (...) {}
  }

  void recovered(const std::string & key, const std::string & message) noexcept {
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it=states_.find(key);
      if (it==states_.end() || !it->second.active) return;
      const auto seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-it->second.first).count();
      std::ostringstream line;
      line << timestamp() << " state=RECOVERED " << message
           << " duration_sec=" << std::fixed << std::setprecision(3) << seconds
           << " total_failures=" << it->second.total;
      append(line.str());
      it->second=State{};
    } catch (...) {}
  }
private:
  static std::string timestamp() {
    const auto t=std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{}; localtime_r(&t,&tm);
    std::ostringstream out; out << std::put_time(&tm,"%F %T"); return out.str();
  }
  void write(const char * state,const std::string & message,const State & s,std::uint64_t suppressed) {
    std::ostringstream line;
    line << timestamp() << " state=" << state << " " << message;
    if (state[0]=='S') line << " suppressed=" << suppressed << " total=" << s.total;
    append(line.str());
  }
  void append(const std::string & line) { std::ofstream out(path_,std::ios::app); if(out) out << line << '\n'; }
  std::string path_; double interval_; std::mutex mutex_; std::unordered_map<std::string,State> states_;
};
}
