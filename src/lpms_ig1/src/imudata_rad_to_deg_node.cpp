#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
using std::placeholders::_1;

const float r2d = 57.29577951f;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr rpy_deg_publisher;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr angular_vel_deg_publisher;

void MsgCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    geometry_msgs::msg::Vector3 angular_vel;
    angular_vel.x = msg->angular_velocity.x*r2d;
    angular_vel.y = msg->angular_velocity.y*r2d;
    angular_vel.z = msg->angular_velocity.z*r2d;

    angular_vel_deg_publisher->publish(angular_vel);


    tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    geometry_msgs::msg::Vector3 rpy;
    rpy.x = roll*r2d;
    rpy.y = pitch*r2d;
    rpy.z = yaw*r2d;

    rpy_deg_publisher->publish(rpy);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto n = std::make_shared<rclcpp::Node>("imudata_rad_to_deg_node");
    angular_vel_deg_publisher = n->create_publisher<geometry_msgs::msg::Vector3>("angular_vel_deg", 1000);
    rpy_deg_publisher = n->create_publisher<geometry_msgs::msg::Vector3>("rpy_deg", 1000);
    auto quat_subscriber = n->create_subscription<sensor_msgs::msg::Imu>("data", 1000, std::bind(&MsgCallback, _1));

    RCLCPP_INFO(n->get_logger(), "waiting for imu data");
    rclcpp::spin(n);
    rclcpp::shutdown();
    return 0;
}
