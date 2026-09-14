#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float32.hpp>

// Linux SocketCAN headers
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

class UniversalLpmsParserNode : public rclcpp::Node {
private:
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr euler_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temp_pub_;

    rclcpp::TimerBase::SharedPtr publish_timer_;

    // Node Parameters
    std::string channel_;
    double publish_rate_;
    double timestamp_scale_;

    int sock_fd_ = -1;

    // Math Constants
    const double G_TO_MS2 = 9.80665;
    const double DEG_TO_RAD = M_PI / 180.0;
    const double UT_TO_TESLA = 1e-6;

    // IMU Sensor State (written by the CAN rx thread, read by the publish timer)
    struct State {
        double acc_cal_x = 0.0, acc_cal_y = 0.0, acc_cal_z = 0.0;
        double gyro1_align_x = 0.0, gyro1_align_y = 0.0, gyro1_align_z = 0.0;
        double mag_cal_x = 0.0, mag_cal_y = 0.0, mag_cal_z = 0.0;
        double euler_x = 0.0, euler_y = 0.0, euler_z = 0.0;
        double quat_w = 1.0, quat_x = 0.0, quat_y = 0.0, quat_z = 0.0;
        double temperature = 0.0;
    } state_;
    std::mutex state_mtx_;

public:
    UniversalLpmsParserNode() : Node("universal_lpms_parser_node") {
        // Declare + read parameters (defaults track directly to your SocketCAN layout)
        channel_         = this->declare_parameter<std::string>("channel", "can0");
        publish_rate_    = this->declare_parameter<double>("publish_rate", 200.0);
        timestamp_scale_ = this->declare_parameter<double>("timestamp_scale", 0.001);

        RCLCPP_INFO(this->get_logger(),
                    "Initializing C++ Node. Target SocketCAN Channel: %s", channel_.c_str());

        // Setup advertisements
        imu_pub_   = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        mag_pub_   = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);
        euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/imu/euler", 10);
        temp_pub_  = this->create_publisher<std_msgs::msg::Float32>("/imu/temperature", 10);

        // Connect directly to Linux network interface layers
        if (!initSocketCAN()) {
            RCLCPP_FATAL(this->get_logger(), "CRITICAL: Failed to register CAN hardware interface.");
            throw std::runtime_error("SocketCAN initialization failed");
        }

        // Periodic, high-precision decoupled state publisher (executor-driven wall timer)
        auto period = std::chrono::duration<double>(1.0 / publish_rate_);
        publish_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&UniversalLpmsParserNode::publishStateCallback, this));
    }

    ~UniversalLpmsParserNode() {
        if (sock_fd_ >= 0) {
            close(sock_fd_);
            RCLCPP_INFO(this->get_logger(), "Closed SocketCAN interface cleanly.");
        }
    }

    bool initSocketCAN() {
        struct sockaddr_can addr;
        struct ifreq ifr;

        // Open raw CAN socket execution paths
        if ((sock_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation failed.");
            return false;
        }

        // Bind interface names
        std::strncpy(ifr.ifr_name, channel_.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        if (ioctl(sock_fd_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_ERROR(this->get_logger(),
                         "IOCTL lookup failed targeting interface device: %s", channel_.c_str());
            close(sock_fd_);
            sock_fd_ = -1;
            return false;
        }

        std::memset(&addr, 0, sizeof(addr));
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(sock_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(),
                         "Binding failed targeting SocketCAN network interface: %s", channel_.c_str());
            close(sock_fd_);
            sock_fd_ = -1;
            return false;
        }

        // Configure 10ms read timeout (prevents the rx thread from locking on shutdown)
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 10000;
        setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));

        RCLCPP_INFO(this->get_logger(),
                    "Successfully bound native SocketCAN interface wrapper to %s", channel_.c_str());
        return true;
    }

    void receiveLoop() {
        struct can_frame frame;
        while (rclcpp::ok()) {
            int nbytes = read(sock_fd_, &frame, sizeof(struct can_frame));

            if (nbytes < 0) {
                // Catch timeouts safely so we re-check rclcpp::ok()
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                }
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "Error encountered checking system driver CAN socket stream buffers.");
                continue;
            }

            if (nbytes == sizeof(struct can_frame)) {
                // Drop malformed data streams
                if (frame.can_dlc == 8) {
                    parseFrame(frame.can_id, frame.data);
                }
            }
        }
    }

    void parseFrame(canid_t can_id, const uint8_t* data) {
        // Little-Endian structured short integer layouts (equivalent to struct.unpack("<hhhh"))
        int16_t raw[4];
        std::memcpy(raw, data, 8);

        std::lock_guard<std::mutex> lock(state_mtx_);
        switch (can_id) {
            case 0x181:
                state_.acc_cal_x     = raw[0] * (1.0 / 1000.0);
                state_.acc_cal_y     = raw[1] * (1.0 / 1000.0);
                state_.acc_cal_z     = raw[2] * (1.0 / 1000.0);
                state_.gyro1_align_x = raw[3] * (1.0 / 10.0);
                break;
            case 0x281:
                state_.gyro1_align_y = raw[0] * (1.0 / 10.0);
                state_.gyro1_align_z = raw[1] * (1.0 / 10.0);
                state_.mag_cal_x     = raw[2] * (1.0 / 100.0);
                state_.mag_cal_y     = raw[3] * (1.0 / 100.0);
                break;
            case 0x381:
                state_.mag_cal_z     = raw[0] * (1.0 / 100.0);
                state_.euler_x       = raw[1] * (1.0 / 100.0);
                state_.euler_y       = raw[2] * (1.0 / 100.0);
                state_.euler_z       = raw[3] * (1.0 / 100.0);
                break;
            case 0x481:
                state_.quat_w        = raw[0] * (1.0 / 10000.0);
                state_.quat_x        = raw[1] * (1.0 / 10000.0);
                state_.quat_y        = raw[2] * (1.0 / 10000.0);
                state_.quat_z        = raw[3] * (1.0 / 10000.0);
                break;
            default:
                break;
        }
    }

    void publishStateCallback() {
        // Snapshot the shared state under lock, then build messages lock-free
        State s;
        {
            std::lock_guard<std::mutex> lock(state_mtx_);
            s = state_;
        }

        rclcpp::Time now = this->now();
        const std::string frame_id = "imu_link";

        // 1. Standard IMU message
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = now;
        imu_msg.header.frame_id = frame_id;

        imu_msg.linear_acceleration.x = s.acc_cal_x * G_TO_MS2;
        imu_msg.linear_acceleration.y = s.acc_cal_y * G_TO_MS2;
        imu_msg.linear_acceleration.z = s.acc_cal_z * G_TO_MS2;

        imu_msg.angular_velocity.x = s.gyro1_align_x * DEG_TO_RAD;
        imu_msg.angular_velocity.y = s.gyro1_align_y * DEG_TO_RAD;
        imu_msg.angular_velocity.z = s.gyro1_align_z * DEG_TO_RAD;

        imu_msg.orientation.w = s.quat_w;
        imu_msg.orientation.x = s.quat_x;
        imu_msg.orientation.y = s.quat_y;
        imu_msg.orientation.z = s.quat_z;
        imu_pub_->publish(imu_msg);

        // 2. Magnetometer message
        sensor_msgs::msg::MagneticField mag_msg;
        mag_msg.header.stamp = now;
        mag_msg.header.frame_id = frame_id;
        mag_msg.magnetic_field.x = s.mag_cal_x * UT_TO_TESLA;
        mag_msg.magnetic_field.y = s.mag_cal_y * UT_TO_TESLA;
        mag_msg.magnetic_field.z = s.mag_cal_z * UT_TO_TESLA;
        mag_pub_->publish(mag_msg);

        // 3. Supplementary Euler message
        geometry_msgs::msg::Vector3 euler_msg;
        euler_msg.x = s.euler_x;
        euler_msg.y = s.euler_y;
        euler_msg.z = s.euler_z;
        euler_pub_->publish(euler_msg);

        // 4. Auxiliary temperature output
        if (s.temperature != 0.0) {
            std_msgs::msg::Float32 temp_msg;
            temp_msg.data = static_cast<float>(s.temperature);
            temp_pub_->publish(temp_msg);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto node = std::make_shared<UniversalLpmsParserNode>();

        // Run the blocking SocketCAN read loop in its own thread while the
        // executor services the publish timer.
        std::thread rx_thread([node]() { node->receiveLoop(); });

        rclcpp::spin(node);

        if (rx_thread.joinable()) {
            rx_thread.join();
        }
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("universal_lpms_parser_node"),
                     "Startup failed: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}
