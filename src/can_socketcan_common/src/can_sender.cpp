#include "can_socketcan_common/can_sender.hpp"
#include "rate_limited_logger.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <sstream>
namespace can_socketcan_common {
CanSender::CanSender(const std::string&i,const std::string&p,double interval):interface_name_(i),error_log_path_(p),logger_(std::make_unique<detail::RateLimitedLogger>(p,interval)){socket_fd_=::socket(PF_CAN,SOCK_RAW|SOCK_CLOEXEC,CAN_RAW);if(socket_fd_<0){log_error("socket",errno,strerror(errno));return;}ifreq q{};strncpy(q.ifr_name,i.c_str(),IFNAMSIZ-1);if(ioctl(socket_fd_,SIOCGIFINDEX,&q)<0){log_error("ioctl",errno,strerror(errno));close(socket_fd_);socket_fd_=-1;return;}sockaddr_can a{};a.can_family=AF_CAN;a.can_ifindex=q.ifr_ifindex;if(bind(socket_fd_,reinterpret_cast<sockaddr*>(&a),sizeof(a))<0){log_error("bind",errno,strerror(errno));close(socket_fd_);socket_fd_=-1;}}
CanSender::~CanSender(){if(socket_fd_>=0)close(socket_fd_);}bool CanSender::is_open()const noexcept{return socket_fd_>=0;}
bool CanSender::send_frame(uint32_t id,const std::array<uint8_t,8>&d,uint8_t dlc,bool ext){if(socket_fd_<0||dlc>8||id>(ext?CAN_EFF_MASK:CAN_SFF_MASK)){log_error("validate/write",EINVAL,"invalid socket/frame",true,id);return false;}can_frame f{};f.can_id=id|(ext?CAN_EFF_FLAG:0);f.can_dlc=dlc;memcpy(f.data,d.data(),dlc);if(write(socket_fd_,&f,sizeof(f))!=sizeof(f)){log_error("write",errno,strerror(errno),true,id);return false;}log_recovery(id);return true;}
bool CanSender::send_float32(uint32_t id,float v,bool ext){uint32_t b;memcpy(&b,&v,4);std::array<uint8_t,8>d{};for(int i=0;i<4;++i)d[i]=(b>>(8*i))&0xff;return send_frame(id,d,4,ext);}
void CanSender::log_error(const char*o,int e,const std::string&d,bool h,uint32_t id)const noexcept{std::ostringstream m;m<<"component=can_sender interface="<<interface_name_<<" operation="<<o<<" errno="<<e<<" detail="<<d;if(h)m<<" can_id=0x"<<std::hex<<id;logger_->error(h?std::to_string(id):std::string("socket"),m.str());}
void CanSender::log_recovery(uint32_t id)const noexcept{std::ostringstream m;m<<"component=can_sender interface="<<interface_name_<<" can_id=0x"<<std::hex<<id;logger_->recovered(std::to_string(id),m.str());}
}
