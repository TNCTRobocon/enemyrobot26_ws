#pragma once
#include <array>
#include <cstdint>
#include <memory>
#include <string>
namespace can_socketcan_common { namespace detail { class RateLimitedLogger; }
struct ReceivedCanFrame{std::uint32_t can_id{0};std::uint8_t dlc{0};std::array<std::uint8_t,8> data{};bool extended{false};bool remote_request{false};};
enum class ReceiveStatus{FrameReceived,Timeout,Error};
class CanReceiver{public:
 explicit CanReceiver(const std::string & interface_name="can0",const std::string & error_log_path="/tmp/can_receiver_error.log",double error_log_interval_sec=1.0);
 ~CanReceiver();bool is_open()const noexcept;ReceiveStatus receive(ReceivedCanFrame&,int=-1);
private:
 int socket_fd_{-1};std::string interface_name_,error_log_path_;std::unique_ptr<detail::RateLimitedLogger> logger_;
 void log_error(const char*,int,const std::string&)const noexcept;void log_recovery()const noexcept;
}; }
