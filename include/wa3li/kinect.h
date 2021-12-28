#pragma once

#include <libfreenect/libfreenect.h>

#include <chrono>
#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class KinectNode : public rclcpp::Node
{
public:
    KinectNode(const KinectNode& other) = delete;
    void operator=(const KinectNode&) = delete;

    static KinectNode* create();
    static KinectNode* get_kinect() { return m_kinect; }
    void depth_callback(void* data);
    void rgb_callback(void* data);

    ~KinectNode();

private:
    KinectNode(freenect_context* const f_ctx, freenect_device* const f_dev);

    void timer_callback();

    static KinectNode* m_kinect;
    freenect_context* const m_f_ctx;
    freenect_device* const m_f_dev;
    rclcpp::TimerBase::SharedPtr m_timer;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr m_laser_scan_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_depth_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_rgb_publisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_ir_publisher;
};
