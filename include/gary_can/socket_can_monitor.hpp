#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include <map>
#include <vector>


namespace driver {
namespace can {

    class SocketCANMonitor : public rclcpp::Node {

    public:
        SocketCANMonitor();
        void loop();

    private:
        void open_socket();
        void monitor();

        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher;
        rclcpp::TimerBase::SharedPtr timer;
        bool is_opened;
        int _socket;
        std::map<int, uint64_t> ifindex_recv_cnt;
        std::map<int, uint64_t> last_ifindex_recv_cnt;
        std::vector<std::string> known_if_names;
    };
}
}