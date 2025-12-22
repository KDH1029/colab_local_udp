#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <rcutils/error_handling.h>

#include "std_msgs/msg/float64_multi_array.hpp"

#define PORT 8888
#define MAX_BUF_SIZE 65535

class UdpReceiver : public rclcpp::Node {
public:
    UdpReceiver() : Node("udp_receiver_node") {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed!");
            exit(EXIT_FAILURE);
        }

        struct sockaddr_in serv_addr;
        memset(&serv_addr, 0, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_addr.s_addr = INADDR_ANY;
        serv_addr.sin_port = htons(PORT);

        if (bind(sockfd_, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket bind failed (Port %d)", PORT);
            close(sockfd_);
            exit(EXIT_FAILURE);
        }

        double_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "udp_double", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1), 
            std::bind(&UdpReceiver::udp_receive_loop, this));

        RCLCPP_INFO(this->get_logger(), "UDP Receiver Started on Port %d", PORT);
    }

    ~UdpReceiver() {
        if (sockfd_ >= 0) close(sockfd_);
    }

private:
    void udp_receive_loop() {
        uint8_t buffer[MAX_BUF_SIZE];
        struct sockaddr_in cli_addr;
        socklen_t len = sizeof(cli_addr);

        ssize_t n = recvfrom(sockfd_, buffer, MAX_BUF_SIZE, MSG_DONTWAIT, (struct sockaddr *)&cli_addr, &len);

        if (n > 0) {
            rclcpp::SerializedMessage serialized_msg(n);
            rcutils_uint8_array_t * rcl_array = &serialized_msg.get_rcl_serialized_message();
            
            if (rcl_array->buffer_capacity < (size_t)n) {
                if (rcutils_uint8_array_resize(rcl_array, n) != RCUTILS_RET_OK) {
                    RCLCPP_ERROR(this->get_logger(), "Buffer resize error.");
                    return;
                }
            }
            
            memcpy(rcl_array->buffer, buffer, n);
            rcl_array->buffer_length = n;

            auto msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
            try {
                serializer_.deserialize_message(&serialized_msg, msg.get());
                double_pub_->publish(*msg);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Deserialization Error: %s", e.what());
            }
        }
    }

    int sockfd_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr double_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Serialization<std_msgs::msg::Float64MultiArray> serializer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpReceiver>());
    rclcpp::shutdown();
    return 0;
}