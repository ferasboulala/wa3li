#pragma once

#include <opencv2/opencv.hpp>

#include "libfreenect/libfreenect.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "wa3li_protocol/srv/get_led.hpp"
#include "wa3li_protocol/srv/get_tilt.hpp"
#include "wa3li_protocol/srv/set_led.hpp"
#include "wa3li_protocol/srv/set_tilt.hpp"

class KinectNode : public rclcpp::Node
{
public:
    KinectNode(const KinectNode& other) = delete;
    void operator=(const KinectNode&) = delete;

    static KinectNode* create();
    static KinectNode* get_kinect() { return m_kinect; }

    void depth_callback(void* data);
    void rgb_callback(void* data);

    void get_tilt_callback(const std::shared_ptr<wa3li_protocol::srv::GetTilt::Request> request,
                           std::shared_ptr<wa3li_protocol::srv::GetTilt::Response> response);
    void set_tilt_callback(const std::shared_ptr<wa3li_protocol::srv::SetTilt::Request> request,
                           std::shared_ptr<wa3li_protocol::srv::SetTilt::Response> response);
    void get_led_callback(const std::shared_ptr<wa3li_protocol::srv::GetLed::Request> request,
                          std::shared_ptr<wa3li_protocol::srv::GetLed::Response> response);
    void set_led_callback(const std::shared_ptr<wa3li_protocol::srv::SetLed::Request> request,
                          std::shared_ptr<wa3li_protocol::srv::SetLed::Response> response);

    ~KinectNode();

private:
    KinectNode(freenect_context* const f_ctx, freenect_device* const f_dev);

    [[nodiscard]] bool imu_callback();
    void topics_timer_callback();
    void parameters_timer_callback();

    static KinectNode* m_kinect;
    freenect_context* const m_f_ctx;
    freenect_device* const m_f_dev;
    rclcpp::TimerBase::SharedPtr m_topics_timer;
    rclcpp::TimerBase::SharedPtr m_parameters_timer;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_laser_scan_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_depth_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_rgb_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_ir_publisher;

    rclcpp::Service<wa3li_protocol::srv::GetTilt>::SharedPtr m_get_tilt_service;
    rclcpp::Service<wa3li_protocol::srv::SetTilt>::SharedPtr m_set_tilt_service;
    rclcpp::Service<wa3li_protocol::srv::GetLed>::SharedPtr m_get_led_service;
    rclcpp::Service<wa3li_protocol::srv::SetLed>::SharedPtr m_set_led_service;

    double m_height;
    double m_angle;

    freenect_led_options m_led_color;
};
