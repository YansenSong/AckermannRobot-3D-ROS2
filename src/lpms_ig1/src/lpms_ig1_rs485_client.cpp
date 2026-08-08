#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <cstdlib>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("lpms_ig1_rs485_client");
    auto client = node->create_client<std_srvs::srv::Trigger>("/get_imu_data");

    rclcpp::Rate loop_rate(100);
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

    //wait for service
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the imu service. Exiting.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "IMU service not available, waiting again...");
    }

    int count = 0;
    while (rclcpp::ok())
    {
        auto res = client->async_send_request(req);
        if (rclcpp::spin_until_future_complete(node, res) == 
                rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "get_imu_data: %d", count++);
        }
        else 
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to call service get_imu_data");
        }

        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}