#pragma once

#include <string>
#include <functional>
#include "linux/can.h"
#include "net/if.h"

namespace driver {
namespace can {

class SocketCANSender {

public:
    explicit SocketCANSender(const std::string& ifname);

    ~SocketCANSender();

    bool send(struct can_frame& tx_frame);

private:
    bool is_opened;
    std::string _ifname;
    int _socket;
};

} // namespace driver
} // namespace can