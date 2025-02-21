#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <boost/asio.hpp>
#include <vector>
#include <cstring>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include <thread>
#include <fcntl.h>
#include <stdexcept>
#include <cmath>

using namespace boost::asio;
using namespace std::chrono_literals;

#define START_FRAME 0xABCD
#define DEFAULT_SERIAL_PORT "/dev/ttyUSB0"
#define BAUD_RATE 115200
#define WHEEL_BASE 0.5    // расстояние между колесами, м
#define WHEEL_RADIUS 0.1  // радиус колеса, м

#pragma pack(push, 1)
struct SerialCommand {
    uint16_t start = START_FRAME;
    int16_t steer = 0;
    int16_t speed = 0;
    uint16_t checksum;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct SerialFeedback {
    uint16_t start;
    int16_t  cmd1;
    int16_t  cmd2;
    int16_t  speedR_meas;
    int16_t  speedL_meas;
    int16_t  batVoltage;
    int16_t  boardTemp;
    uint16_t cmdLed;
    uint16_t checksum;
};
#pragma pack(pop)

class HoverboardNode : public rclcpp::Node {
public:
    HoverboardNode()
    : Node("hoverboard_node"),
      serial(io),
      x_(0.0), y_(0.0), theta_(0.0),
      feedback_timeout_sec_(2.0)
    {
        this->declare_parameter<std::string>("serial_port", DEFAULT_SERIAL_PORT);
        this->get_parameter("serial_port", serial_port_);

        // Подписка на cmd_vel
        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&HoverboardNode::cmdVelCallback, this, std::placeholders::_1));
        // Паблишеры обратной связи, одометрии и статуса
        feedback_pub = this->create_publisher<std_msgs::msg::Int16MultiArray>("hoverboard_feedback", 10);
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("hoverboard_status", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        openSerial();
        startAsyncRead();
        io_thread_ = std::thread([this]() { io.run(); });

        // Таймер отправки команд - 10 Гц
        send_timer_ = this->create_wall_timer(100ms,
            std::bind(&HoverboardNode::sendCommandTimerCallback, this));
        // Таймер проверки связи - каждые 500 мс
        connection_check_timer_ = this->create_wall_timer(500ms,
            std::bind(&HoverboardNode::checkConnectionCallback, this));

        // Инициализируем время получения обратной связи текущим временем
        last_feedback_time_ = this->now();
        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Hoverboard node started");
    }

    ~HoverboardNode() override {
        io.stop();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
        if (serial.is_open()) {
            serial.close();
        }
    }

private:
    // Boost.Asio
    io_service io;
    serial_port serial;
    std::thread io_thread_;

    // ROS2 подписчики и паблишеры
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr feedback_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    rclcpp::TimerBase::SharedPtr connection_check_timer_;

    // Буфер для накопления полученных байт
    std::vector<uint8_t> read_buffer_;

    // Последняя команда, полученная по cmd_vel
    SerialCommand last_command;

    // Переменные для одометрии
    double x_, y_, theta_;
    rclcpp::Time last_time_;

    // Отслеживание времени последнего корректного пакета обратной связи
    rclcpp::Time last_feedback_time_;
    const double feedback_timeout_sec_;

    std::string serial_port_;

    void openSerial() {
        try {
            serial.open(serial_port_);
            serial.set_option(serial_port_base::baud_rate(BAUD_RATE));
            // Устанавливаем неблокирующий режим через нативный дескриптор и fcntl
            int fd = serial.native_handle();
            int flags = fcntl(fd, F_GETFL, 0);
            if (flags == -1) {
                throw std::runtime_error("fcntl(F_GETFL) failed");
            }
            if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {
                throw std::runtime_error("fcntl(F_SETFL) failed");
            }
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully", serial_port_.c_str());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
        }
        last_time_ = this->now();
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear.x=%.2f, angular.z=%.2f",
                    msg->linear.x, msg->angular.z);
        // Преобразуем линейную и угловую скорости в значения для команды
        last_command.steer = static_cast<int16_t>(msg->angular.z * 100);
        last_command.speed = static_cast<int16_t>(msg->linear.x * 300);
        last_command.checksum = last_command.start ^ last_command.steer ^ last_command.speed;
    }

    void sendCommandTimerCallback() {
        try {
            boost::system::error_code ec;
            write(serial, buffer(&last_command, sizeof(last_command)), ec);
            if (ec) {
                RCLCPP_ERROR(this->get_logger(), "Serial write error: %s", ec.message().c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Sending command: steer=%d, speed=%d",
                            last_command.steer, last_command.speed);
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in sendCommandTimerCallback: %s", e.what());
        }
    }

    void startAsyncRead() {
        auto buf = std::make_shared<std::vector<uint8_t>>(64);
        serial.async_read_some(boost::asio::buffer(*buf),
            [this, buf](const boost::system::error_code &ec, std::size_t bytes_transferred) {
                handleSerialRead(ec, bytes_transferred, buf);
            });
    }

    void handleSerialRead(const boost::system::error_code &ec, std::size_t bytes_transferred,
                          std::shared_ptr<std::vector<uint8_t>> buf) {
        if (ec) {
            if (ec == boost::asio::error::would_block || ec == boost::asio::error::try_again) {
                // Если данных нет, запланировать повторное чтение через небольшую задержку
                auto timer = std::make_shared<boost::asio::steady_timer>(io, std::chrono::milliseconds(10));
                timer->async_wait([this, timer](const boost::system::error_code &) {
                    startAsyncRead();
                });
            } else {
                RCLCPP_ERROR(this->get_logger(), "Serial read error: %s", ec.message().c_str());
                startAsyncRead();
            }
            return;
        }
        // Добавляем полученные данные в наш буфер
        read_buffer_.insert(read_buffer_.end(), buf->begin(), buf->begin() + bytes_transferred);
        processReadBuffer();
        startAsyncRead();
    }

    void processReadBuffer() {
        const size_t packet_size = sizeof(SerialFeedback);
        while (read_buffer_.size() >= packet_size) {
            bool found = false;
            size_t pos = 0;
            // Поиск стартовой метки в little-endian порядке:
            for (; pos + 1 < read_buffer_.size(); ++pos) {
                uint16_t frame = (read_buffer_[pos + 1] << 8) | read_buffer_[pos];
                if (frame == START_FRAME) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                read_buffer_.clear();
                break;
            }
            if (read_buffer_.size() - pos < packet_size) {
                if (pos > 0)
                    read_buffer_.erase(read_buffer_.begin(), read_buffer_.begin() + pos);
                break;
            }
            SerialFeedback packet;
            std::memcpy(&packet, &read_buffer_[pos], packet_size);
            uint16_t calculated_checksum = packet.start ^ packet.cmd1 ^ packet.cmd2 ^
                packet.speedR_meas ^ packet.speedL_meas ^ packet.batVoltage ^ packet.boardTemp ^ packet.cmdLed;
            if (packet.start == START_FRAME && calculated_checksum == packet.checksum) {
                RCLCPP_INFO(this->get_logger(), "Valid feedback received");
                publishFeedback(packet);
                read_buffer_.erase(read_buffer_.begin(), read_buffer_.begin() + pos + packet_size);
            } else {
                // Если пакет некорректен – удаляем один байт и продолжаем поиск
                read_buffer_.erase(read_buffer_.begin());
            }
        }
    }

    void publishFeedback(const SerialFeedback &feedback) {
        std_msgs::msg::Int16MultiArray msg;
        msg.data = {feedback.cmd1, feedback.cmd2, feedback.speedR_meas, feedback.speedL_meas,
                    feedback.batVoltage, feedback.boardTemp, static_cast<int16_t>(feedback.cmdLed)};
        feedback_pub->publish(msg);

        // Обновляем время получения корректного пакета
        last_feedback_time_ = this->now();

        // Расчёт одометрии.
        // Для прямолинейного движения один мотор вращается в обратную сторону.
        double v_right = (feedback.speedR_meas / 100.0) * WHEEL_RADIUS;
        double v_left  = -(feedback.speedL_meas / 100.0) * WHEEL_RADIUS;
        double v = (v_right + v_left) / 2.0;
        double w = (v_right - v_left) / WHEEL_BASE;

        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        x_ += v * dt * cos(theta_);
        y_ += v * dt * sin(theta_);
        theta_ += w * dt;

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.twist.twist.linear.x = v;
        odom_msg.twist.twist.angular.z = w;
        odom_pub_->publish(odom_msg);

        // Публикуем TF
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp = current_time;
        odom_tf.header.frame_id = "odom";
        odom_tf.child_frame_id = "base_link";
        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;
        odom_tf.transform.rotation.x = q.x();
        odom_tf.transform.rotation.y = q.y();
        odom_tf.transform.rotation.z = q.z();
        odom_tf.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(odom_tf);
    }

    void checkConnectionCallback() {
        rclcpp::Time now = this->now();
        double dt = (now - last_feedback_time_).seconds();
        if (dt > feedback_timeout_sec_) {
            RCLCPP_ERROR(this->get_logger(), "No feedback received for %.2f seconds!", dt);
            std_msgs::msg::String status_msg;
            status_msg.data = "Error: Hoverboard not responding!";
            status_pub_->publish(status_msg);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HoverboardNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
