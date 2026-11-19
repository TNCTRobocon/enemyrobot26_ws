#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <string>
namespace can_socketcan_common { namespace detail { class RateLimitedLogger; }
class CanSender { public:
 explicit CanSender(const std::string & interface_name="can0",const std::string & error_log_path="/tmp/can_sender_error.log",double error_log_interval_sec=1.0);
 ~CanSender(); bool is_open() const noexcept;
 bool send_frame(std::uint32_t,const std::array<std::uint8_t,8>&,std::uint8_t=8,bool=false);
 bool send_float32(std::uint32_t,float,bool=false);
private:
 int socket_fd_{-1}; std::string interface_name_,error_log_path_; std::unique_ptr<detail::RateLimitedLogger> logger_;
 void log_error(const char*,int,const std::string&,bool=false,std::uint32_t=0) const noexcept;
 void log_recovery(std::uint32_t) const noexcept;
}; }
