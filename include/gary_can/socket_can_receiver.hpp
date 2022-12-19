#pragma once

#include <string>
#include <map>
#include "linux/can.h"
#include "net/if.h"

namespace driver {
namespace can {

class SocketCANReceiver {

public:
    explicit SocketCANReceiver(const std::string& ifname);

    ~SocketCANReceiver();

    bool bind(uint32_t frame_id);

    bool read(uint32_t frame_id, struct can_frame *frame);

private:
    std::string _ifname;
    std::map<uint32_t, int> _sockets;
};

} // namespace driver
} // namespace can