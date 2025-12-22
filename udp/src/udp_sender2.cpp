#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

#include "std_msgs/msg/float64_multi_array.hpp"

#define IP "127.0.0.1" 
#define PORT 8888

using std::placeholders::_1;

class UdpSender : public rclcpp::Node {
public:
    UdpSender() : Node("udp_sender_node") {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed!");
            return;
        }

        memset(&dest_addr_, 0, sizeof(dest_addr_));
        dest_addr_.sin_family = AF_INET;
        dest_addr_.sin_port = htons(PORT);
        dest_addr_.sin_addr.s_addr = inet_addr(IP);

        double_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "udp_double", 
            10, 
            std::bind(&UdpSender::double_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "UDP Sender Started. Target: %s:%d", IP, PORT);
    }

    ~UdpSender() {
        if (sockfd_ >= 0) close(sockfd_);
    }

private:
    void double_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        rclcpp::SerializedMessage serialized_msg;
        serializer_.serialize_message(msg.get(), &serialized_msg);

        auto buffer_ptr = serialized_msg.get_rcl_serialized_message().buffer;
        auto buffer_len = serialized_msg.get_rcl_serialized_message().buffer_length;

        ssize_t sent_bytes = sendto(sockfd_, buffer_ptr, buffer_len, 0, (struct sockaddr *)&dest_addr_, sizeof(dest_addr_));

        if (sent_bytes < 0) {
            RCLCPP_WARN(this->get_logger(), "UDP Send Failed");
        }
    }

    int sockfd_;
    struct sockaddr_in dest_addr_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr double_sub_;
    rclcpp::Serialization<std_msgs::msg::Float64MultiArray> serializer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpSender>());
    rclcpp::shutdown();
    return 0;
}