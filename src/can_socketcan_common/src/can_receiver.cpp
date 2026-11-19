#include "can_socketcan_common/can_receiver.hpp"
#include "rate_limited_logger.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <sstream>
namespace can_socketcan_common {
CanReceiver::CanReceiver(const std::string&i,const std::string&p,double interval):interface_name_(i),error_log_path_(p),logger_(std::make_unique<detail::RateLimitedLogger>(p,interval)){socket_fd_=socket(PF_CAN,SOCK_RAW|SOCK_CLOEXEC,CAN_RAW);if(socket_fd_<0){log_error("socket",errno,strerror(errno));return;}ifreq q{};strncpy(q.ifr_name,i.c_str(),IFNAMSIZ-1);if(ioctl(socket_fd_,SIOCGIFINDEX,&q)<0){log_error("ioctl",errno,strerror(errno));close(socket_fd_);socket_fd_=-1;return;}sockaddr_can a{};a.can_family=AF_CAN;a.can_ifindex=q.ifr_ifindex;if(bind(socket_fd_,reinterpret_cast<sockaddr*>(&a),sizeof(a))<0){log_error("bind",errno,strerror(errno));close(socket_fd_);socket_fd_=-1;}}
CanReceiver::~CanReceiver(){if(socket_fd_>=0)close(socket_fd_);}bool CanReceiver::is_open()const noexcept{return socket_fd_>=0;}
ReceiveStatus CanReceiver::receive(ReceivedCanFrame&o,int ms){if(socket_fd_<0)return ReceiveStatus::Error;pollfd p{socket_fd_,POLLIN,0};int r=poll(&p,1,ms);if(r==0)return ReceiveStatus::Timeout;if(r<0){log_error("poll",errno,strerror(errno));return ReceiveStatus::Error;}can_frame f{};if(read(socket_fd_,&f,sizeof(f))!=sizeof(f)){log_error("read",errno,strerror(errno));return ReceiveStatus::Error;}o.extended=f.can_id&CAN_EFF_FLAG;o.remote_request=f.can_id&CAN_RTR_FLAG;o.can_id=o.extended?(f.can_id&CAN_EFF_MASK):(f.can_id&CAN_SFF_MASK);o.dlc=f.can_dlc;memcpy(o.data.data(),f.data,o.dlc);log_recovery();return ReceiveStatus::FrameReceived;}
void CanReceiver::log_error(const char*o,int e,const std::string&d)const noexcept{std::ostringstream m;m<<"component=can_receiver interface="<<interface_name_<<" operation="<<o<<" errno="<<e<<" detail="<<d;logger_->error("receive",m.str());}
void CanReceiver::log_recovery()const noexcept{logger_->recovered("receive","component=can_receiver interface="+interface_name_);}
}
