#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

#include "humanoid_interfaces/msg/robocupvision25.hpp"

#define ROBOT_B_IP "127.0.0.1" 
#define UDP_PORT 8888

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
        dest_addr_.sin_port = htons(UDP_PORT);
        dest_addr_.sin_addr.s_addr = inet_addr(ROBOT_B_IP);

        vision_sub_ = this->create_subscription<humanoid_interfaces::msg::Robocupvision25>(
            "vision", 
            10, 
            std::bind(&UdpSender::vision_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "UDP Sender Started. Target: %s:%d", ROBOT_B_IP, UDP_PORT);
    }

    ~UdpSender() {
        if (sockfd_ >= 0) close(sockfd_);
    }

private:
    void vision_callback(const humanoid_interfaces::msg::Robocupvision25::SharedPtr msg) {
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
    rclcpp::Subscription<humanoid_interfaces::msg::Robocupvision25>::SharedPtr vision_sub_;
    rclcpp::Serialization<humanoid_interfaces::msg::Robocupvision25> serializer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpSender>());
    rclcpp::shutdown();
    return 0;
}