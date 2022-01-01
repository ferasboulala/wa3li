#pragma once

#include <memory>
#include <optional>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kobuki/kobuki.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

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
    void publish_imu(const kobuki::GyroData& gyro_data);
    void publisher_odometry_data(const kobuki::BasicData& basic_data);
    void publish_general_sensor_data(const kobuki::BasicData& basic_data);
    void publish_battery_data(const kobuki::BasicData& basic_data);

    static KobukiNode* m_kobuki;
    std::unique_ptr<kobuki::Kobuki> m_driver;
    rclcpp::TimerBase::SharedPtr m_timer;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr m_transform_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_twist_publisher;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr m_battery_state_publisher;

    std::optional<uint16_t> m_left_encoder;
    std::optional<uint16_t> m_right_encoder;
    std::optional<uint16_t> m_prev_timestamp_ms;
};
