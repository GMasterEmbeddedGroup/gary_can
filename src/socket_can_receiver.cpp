#include "gary_can/socket_can_receiver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sys/socket.h>
#include <cstring>
#include <utility>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <fcntl.h>

using namespace driver::can;


SocketCANReceiver::SocketCANReceiver(const std::string &ifname) {
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "creating socket_can_receiver");
    this->_ifname = ifname;
}


SocketCANReceiver::~SocketCANReceiver() {
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "destroying socket_can_receiver");

    //close all sockets
    for (auto i: this->_sockets) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[socket %d] closing socket for frame id 0x%X",
                     i.second, i.first);
        close(i.second);
    }
}


bool SocketCANReceiver::bind(uint32_t frame_id) {

    //create socket
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[interface %s id 0x%X] creating socket",
                 this->_ifname.c_str(), frame_id);
    int _socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    //select interface
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[interface %s id 0x%X] setting interface index",
                 this->_ifname.c_str(), frame_id);
    struct ifreq _ifreq{};
    strcpy(_ifreq.ifr_name, this->_ifname.c_str());
    if (int ret = ioctl(_socket, SIOCGIFINDEX, &_ifreq) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("socket_can_receiver"),
                     "[interface %s id 0x%X errno %d ret %d] failed to select interface",
                     this->_ifname.c_str(), frame_id, ret);
        return false;
    }

    //bind
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[interface %s id 0x%X] binding",
                 this->_ifname.c_str(), frame_id);
    struct sockaddr_can _sockaddr_can{};
    _sockaddr_can.can_family = AF_CAN;
    _sockaddr_can.can_ifindex = _ifreq.ifr_ifindex;
    if (int ret = ::bind(_socket, (struct sockaddr *) &_sockaddr_can, sizeof(_sockaddr_can)) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("socket_can_receiver"),
                     "[interface %s id 0x%X errno %d ret %d] failed to bind to interface",
                     this->_ifname.c_str(), frame_id, errno, ret);
        return false;
    }

    //set filter
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[interface %s id 0x%X] setting can filter",
                 this->_ifname.c_str(), frame_id);
    struct can_filter _can_filter[1];
    _can_filter[0].can_id = frame_id;
    _can_filter[0].can_mask = 0xffff;
    _can_filter[0].can_mask = CAN_SFF_MASK;
    if (int ret = setsockopt(_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &_can_filter, sizeof(_can_filter)) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("socket_can_receiver"),
                     "[interface %s id 0x%X errno %d ret %d] failed to set can filter",
                     this->_ifname.c_str(), frame_id, errno, ret);
        return false;
    }

    //nonblocking mode
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[interface %s id 0x%X] setting nonblocking mode",
                 this->_ifname.c_str(), frame_id);
    int flags = fcntl(_socket, F_GETFL, 0);
    if (int ret = fcntl(_socket, F_SETFL, flags | O_NONBLOCK) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("socket_can_receiver"),
                     "[interface %s id 0x%X errno %d ret %d] failed to set nonblocking mode",
                     this->_ifname.c_str(), frame_id, errno, ret);
        return false;
    }

    //a socket only receive one frame id, make them a pair
    this->_sockets.insert(std::make_pair(frame_id, _socket));

    return true;
}


bool SocketCANReceiver::read(uint32_t frame_id, struct can_frame *frame) {

    //check frame id is valid
    if (this->_sockets.count(frame_id) == 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[id 0x%X] unbinded frame id", frame_id);
        return false;
    }

    //attempt to read
    long len = ::read(this->_sockets[frame_id], frame, sizeof(can_frame));

    //check return value
    if (len == sizeof(can_frame)) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[id 0x%X dlc %d] receive successful", frame_id,
                     frame->can_dlc);
        return true;
    } else if (len < 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"), "[interface %s return %d errno %d] failed to read",
                     this->_ifname.c_str(), len, errno);
    } else if (len != sizeof(struct can_frame)) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_receiver"),
                     "[interface %s errno %d] attempt to read %d bytes, %d byte read", this->_ifname.c_str(), errno,
                     sizeof(struct can_frame), len);
    }
    return false;
}
