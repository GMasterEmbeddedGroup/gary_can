#include "gary_can/socket_can_sender.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sys/socket.h>
#include <cstring>
#include <utility>
#include <sys/ioctl.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <fcntl.h>

using namespace driver::can;


SocketCANSender::SocketCANSender(const std::string &ifname) {

    this->_ifname = ifname;

    //create socket
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[interface %s] creating socket", this->_ifname.c_str());
    this->_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    //select interface
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[interface %s] setting interface index",
                 this->_ifname.c_str());
    struct ifreq _ifreq{};
    strcpy(_ifreq.ifr_name, this->_ifname.c_str());
    if (int ret = ioctl(_socket, SIOCGIFINDEX, &_ifreq) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("socket_can_sender"),
                     "[interface %s errno %d ret %d] failed to select interface", this->_ifname.c_str(), ret);
        this->is_opened = false;
        return;
    }

    //bind
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[interface %s] binding", this->_ifname.c_str());
    struct sockaddr_can _sockaddr_can{};
    _sockaddr_can.can_family = AF_CAN;
    _sockaddr_can.can_ifindex = _ifreq.ifr_ifindex;
    if (int ret = ::bind(_socket, (struct sockaddr *) &_sockaddr_can, sizeof(_sockaddr_can)) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("socket_can_sender"),
                     "[interface %s errno %d ret %d] failed to bind to interface", this->_ifname.c_str(), errno, ret);
        this->is_opened = false;
        return;
    }

    //nonblocking mode
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[interface %s] setting nonblocking mode",
                 this->_ifname.c_str());
    int flags = fcntl(_socket, F_GETFL, 0);
    if (int ret = fcntl(_socket, F_SETFL, flags | O_NONBLOCK) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("socket_can_sender"),
                     "[interface %s errno %d ret %d] failed to set nonblocking mode",
                     this->_ifname.c_str(), errno, ret);
        this->is_opened = false;
        return;
    }

    //disable can filter
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[interface %s] disabling can filter", this->_ifname.c_str());
    if (int ret = setsockopt(this->_socket, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("socket_can_sender"),
                     "[interface %s errno %d ret %d] failed to disable can filter", this->_ifname.c_str(), errno, ret);
        this->is_opened = false;
        return;
    }

    this->is_opened = true;
}


SocketCANSender::~SocketCANSender() {
    RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "destroying socket_can_sender");

    //close the socket if it is opened
    if (this->is_opened) {
        close(this->_socket);
    }
}


bool SocketCANSender::send(struct can_frame &tx_frame) {
    //check opened
    if (!this->is_opened) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[interface %s] socket is not opened",
                     this->_ifname.c_str());
        return false;
    }

    //attempt to write
    long len = ::write(this->_socket, &tx_frame, sizeof(can_frame));

    //check return value
    if (len == sizeof(struct can_frame)) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[id 0x%X dlc %d] send successful", tx_frame.can_id,
                     tx_frame.can_dlc);
        return true;
    } else if (len < 0) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"), "[interface %s return %d errno %d] failed to write",
                     this->_ifname.c_str(), len, errno);
    } else if (len != sizeof(struct can_frame)) {
        RCLCPP_DEBUG(rclcpp::get_logger("socket_can_sender"),
                     "[interface %s errno %d] attempt to write %d bytes, %d byte written", this->_ifname.c_str(), errno,
                     sizeof(struct can_frame), len);
    }
    return false;
}
