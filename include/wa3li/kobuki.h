#pragma once

#include <memory>
#include <optional>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "kobuki/kobuki.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_echo.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "wa3li_protocol/msg/kobuki_basic_data.hpp"
#include "wa3li_protocol/msg/kobuki_current.hpp"
#include "wa3li_protocol/msg/kobuki_gpi.hpp"
#include "wa3li_protocol/srv/kobuki_power.hpp"
#include "wa3li_protocol/srv/led.hpp"
#include "wa3li_protocol/srv/pid.hpp"
#include "wa3li_protocol/srv/sound_sequence.hpp"

class KobukiNode : public rclcpp::Node
{
public:
    KobukiNode(const KobukiNode& other) = delete;
    void operator=(const KobukiNode&) = delete;

    static KobukiNode* create();
    static KobukiNode* get_kobuki() { return m_kobuki; }

    ~KobukiNode() = default;

private:
    KobukiNode(kobuki::Kobuki* const driver);

    void timer_callback();
    void twist_command_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void publish_imu(const kobuki::GyroData& gyro_data);
    void publisher_odometry_data(const kobuki::BasicData& basic_data);
    void publish_general_sensor_data(const kobuki::BasicData& basic_data);
    void publish_battery_data(const kobuki::BasicData& basic_data);
    void publish_cliff_data(const kobuki::CliffData& cliff_data);
    void publish_current(const kobuki::Current& current);
    void publish_gpi(const kobuki::GeneralPurposeInput& gpi);

    static KobukiNode* m_kobuki;
    std::unique_ptr<kobuki::Kobuki> m_driver;
    rclcpp::TimerBase::SharedPtr m_timer;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_twist_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr m_odom_transform_publisher;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_odom_twist_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_publisher;

    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr m_battery_state_publisher;
    rclcpp::Publisher<sensor_msgs::msg::LaserEcho>::SharedPtr m_cliff_publisher;
    rclcpp::Publisher<wa3li_protocol::msg::KobukiBasicData>::SharedPtr
        m_kobuki_basic_data_publisher;
    rclcpp::Publisher<wa3li_protocol::msg::KobukiCurrent>::SharedPtr m_kobuki_current_publisher;
    rclcpp::Publisher<wa3li_protocol::msg::KobukiGpi>::SharedPtr m_kobuki_gpi_publisher;

    rclcpp::Service<wa3li_protocol::srv::KobukiPower>::SharedPtr m_kobuki_power_service;
    rclcpp::Service<wa3li_protocol::srv::SoundSequence>::SharedPtr m_sound_sequence_service;
    rclcpp::Service<wa3li_protocol::srv::Pid>::SharedPtr m_pid_service;

    std::optional<uint16_t> m_left_encoder;
    std::optional<uint16_t> m_right_encoder;
    std::optional<uint16_t> m_prev_timestamp_ms;
};
