
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
using std::placeholders::_1;

rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rpy_publisher;

void MsgCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    geometry_msgs::msg::Vector3 rpy;
    rpy.x = roll;
    rpy.y = pitch;
    rpy.z = yaw;

    rpy_publisher->publish(rpy);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = std::make_shared<rclcpp::Node>("quat_to_euler_node");
    rpy_publisher = n->create_publisher<geometry_msgs::msg::Vector3>("rpy_angles", 1000);
    auto quat_subscriber = n->create_subscription<sensor_msgs::msg::Imu>("data", 1000, std::bind(&MsgCallback, _1));

    RCLCPP_INFO(n->get_logger(), "waiting for imu data");
    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}
