#include "gary_can/socket_can_monitor.hpp"
#include <sys/socket.h>
#include <cstring>
#include <linux/can/raw.h>
#include <linux/can.h>
#include <net/if.h>
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <libsocketcan.h>


using namespace std::chrono_literals;
using namespace driver::can;


SocketCANMonitor::SocketCANMonitor() : rclcpp::Node("socket_can_monitor") {
    //create publisher
    this->diagnostic_publisher = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",
                                                                                               rclcpp::SystemDefaultsQoS());

    //create timer
    this->timer = this->create_wall_timer(100ms, [this] { monitor(); });

    this->_socket = 0;
    this->is_opened = false;
}


void SocketCANMonitor::loop() {

    //reopen socket if closed
    if (!this->is_opened) {
        this->open_socket();
        return;
    }

    //set time out
    struct timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 100000;    //100ms

    //set fd
    fd_set rd_fd{};
    FD_ZERO(&rd_fd);
    FD_SET(this->_socket, &rd_fd);

    //select
    int ret = select(this->_socket + 1, &rd_fd, nullptr, nullptr, &tv);

    if (ret < 0) {
        //select error
        if (errno == EINTR) {
            //if received interrupt signal
            return;
        } else {
            //other error
            RCLCPP_DEBUG(this->get_logger(), "failed to select, socket %d, return %d, error: %s",
                         this->_socket, ret, std::strerror(errno));
            close(this->_socket);
            this->is_opened = false;
            return;
        }
    } else if (ret == 0) {
        //select time out
        RCLCPP_DEBUG(this->get_logger(), "select timeout");
        return;
    }

    struct sockaddr_can src_addr{};
    socklen_t addrlen;
    struct can_frame frame{};

    //try to read
    long len = recvfrom(this->_socket, &frame, sizeof(struct can_frame), 0, (struct sockaddr *) &src_addr,
                        &addrlen);
    if (len == sizeof(struct can_frame)) {
        //succ
        this->ifindex_recv_cnt[src_addr.can_ifindex]++;
//        RCLCPP_DEBUG(this->get_logger(), "read succ, index %d", src_addr.can_ifindex);
    } else {
        //fail
        close(this->_socket);
        this->is_opened = false;
        RCLCPP_DEBUG(this->get_logger(), "read fail, return %d, error: %s", len, std::strerror(errno));
    }
}


void SocketCANMonitor::open_socket() {
    //create new socket
    RCLCPP_DEBUG(this->get_logger(), "creating socket");
    this->_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    //bind
    RCLCPP_DEBUG(this->get_logger(), "binding");
    struct sockaddr_can _sockaddr_can{};
    _sockaddr_can.can_family = AF_CAN;
    _sockaddr_can.can_ifindex = 0;
    if (int ret = ::bind(this->_socket, (struct sockaddr *) &_sockaddr_can, sizeof(_sockaddr_can)) != 0) {
        RCLCPP_DEBUG(this->get_logger(), "failed to bind to interface, return %d, error: %s",
                     ret, std::strerror(errno));
        this->is_opened = false;
        close(this->_socket);
        return;
    }

    //set filter
    RCLCPP_DEBUG(this->get_logger(), "setting can filter");
    struct can_filter _can_filter[1];
    _can_filter[0].can_id = 0;
    _can_filter[0].can_mask = 0;
    if (int ret = setsockopt(this->_socket, SOL_CAN_RAW, CAN_RAW_FILTER, &_can_filter, sizeof(_can_filter)) != 0) {
        RCLCPP_DEBUG(this->get_logger(), "failed to set can filter, return %d, error: %s",
                     ret, std::strerror(errno));
        this->is_opened = false;
        close(this->_socket);
        return;
    }

    //nonblocking mode
    RCLCPP_DEBUG(this->get_logger(), "setting nonblocking mode");
    int flags = fcntl(_socket, F_GETFL, 0);
    if (int ret = fcntl(_socket, F_SETFL, flags | O_NONBLOCK) != 0) {
        RCLCPP_DEBUG(this->get_logger(), "failed to set nonblocking mode, return %d, error: %s",
                     ret, std::strerror(errno));
        this->is_opened = false;
        close(this->_socket);
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "create socket successful");
    this->is_opened = true;
}


void SocketCANMonitor::monitor() {
    //diagnostic message
    diagnostic_msgs::msg::DiagnosticArray diagnostic_array;

    //get all interface info
    struct if_nameindex *name_index = if_nameindex();
    std::map<std::string, uint32_t> if_info;
    for (int i = 0; i < 10; ++i) {
        if (name_index[i].if_index == 0) break;
        if_info.emplace(std::string(name_index[i].if_name), name_index[i].if_index);
    }
    if_freenameindex(name_index);

    //copy all monitored ifname
    auto if_names = this->known_if_names;

    //foreach all ifindex
    for (const auto &i: this->ifindex_recv_cnt) {

        diagnostic_msgs::msg::DiagnosticStatus diagnostic_status;

        //get ifname
        char ifname[64]{0};
        if_indextoname(i.first, ifname);
        //ifindex does not exist
        if (ifname[0] == '\0') continue;

        diagnostic_status.hardware_id = ifname;
        diagnostic_status.name = ifname;

        //check new if
        auto pos = std::find(if_names.begin(), if_names.end(), std::string(ifname));
        if (pos == if_names.end()) {
            //new if
            RCLCPP_INFO(this->get_logger(), "adding new can interface %s", ifname);
            this->known_if_names.emplace_back(ifname);
        } else {
            //existing if
            if_names.erase(pos);
        }

        //get bitrate
        struct can_bittiming bittiming{};
        if (can_get_bittiming(ifname, &bittiming) != 0) {
            //get bitrate failed
            auto clock = rclcpp::Clock();
            RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "[%s] failed to get bitrate", ifname);
            diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diagnostic_status.message = "failed to get bitrate";
            diagnostic_array.status.emplace_back(diagnostic_status);
            continue;
        }

        //get timer period
        int64_t period;
        if (rcl_timer_get_period(this->timer->get_timer_handle().get(), &period) != RCL_RET_OK) {
            auto clock = rclcpp::Clock();
            RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "[%s] failed to get timer period", ifname);
            diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diagnostic_status.message = "failed to get timer period";
            diagnostic_array.status.emplace_back(diagnostic_status);
            continue;
        }
        //to freq
        double freq = 1.0f / (static_cast<double>(period) / 1000'000'000);

        //calc delta packet
        uint64_t delta_pkt = i.second - this->last_ifindex_recv_cnt[i.first];
        //calc load rate
        int packet_len = 110;
        double load = (static_cast<double>(delta_pkt) * packet_len) / (bittiming.bitrate / freq);

        diagnostic_msgs::msg::KeyValue bus_load;
        bus_load.key = "bus_load";
        bus_load.value = std::to_string(load);
        diagnostic_status.values.emplace_back(bus_load);

        //check if overloaded
        if (load > 0.8) {
            diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diagnostic_status.message = "can bus overload";
            diagnostic_array.status.emplace_back(diagnostic_status);
            auto clock = rclcpp::Clock();
            RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "[%s] can bus overload %f", ifname, load);
            continue;
        }


        int state;
        can_get_state(ifname, &state);
        if (state > CAN_STATE_ERROR_ACTIVE) {
            diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diagnostic_status.message = "transmission jammed";
            diagnostic_array.status.emplace_back(diagnostic_status);
            auto clock = rclcpp::Clock();
            RCLCPP_WARN_THROTTLE(this->get_logger(), clock, 1000, "[%s] transmission jammed, state %d", ifname, state);
            continue;
        }

        //no error detected
        diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        diagnostic_status.message = "ok";
        diagnostic_array.status.emplace_back(diagnostic_status);

        RCLCPP_DEBUG(this->get_logger(), "[%s] ifindex %d, delta pkt %ld, bitrate %d, load %f state %d", ifname, i.first,
                     delta_pkt, bittiming.bitrate, load, state);
    }

    //foreach offline can interface
    for (const auto &i: if_names) {
        diagnostic_msgs::msg::DiagnosticStatus diagnostic_status;
        diagnostic_status.hardware_id = i;
        diagnostic_status.name = i;
        diagnostic_status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diagnostic_status.message = "offline";
        diagnostic_array.status.emplace_back(diagnostic_status);
        auto clock = rclcpp::Clock();
        RCLCPP_ERROR_THROTTLE(this->get_logger(), clock, 1000, "[%s] offline", i.c_str());
    }

    this->last_ifindex_recv_cnt = this->ifindex_recv_cnt;

    //publish
    diagnostic_array.header.frame_id = "";
    diagnostic_array.header.stamp = this->get_clock()->now();
    this->diagnostic_publisher->publish(diagnostic_array);
}

int main(int argc, char const *const argv[]) {
    rclcpp::init(argc, argv);

    auto socket_can_monitor = std::make_shared<SocketCANMonitor>();

    while (rclcpp::ok()) {
        try {
            socket_can_monitor->loop();
            rclcpp::spin_some(socket_can_monitor);
        } catch (const rclcpp::exceptions::RCLError &e) {}
    }

    rclcpp::shutdown();
}