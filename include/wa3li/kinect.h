#include <libfreenect/libfreenect.h>

#include <chrono>
#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

class KinectNode : public rclcpp::Node
{
public:
    KinectNode(const KinectNode& other) = delete;
    void operator=(const KinectNode&) = delete;

    static KinectNode* create();

    ~KinectNode();

private:
    KinectNode(freenect_context* const f_ctx, freenect_device* const f_dev);

    void timer_callback();

    static KinectNode* m_kinect;
    freenect_context* const m_f_ctx;
    freenect_device* const m_f_dev;
    rclcpp::TimerBase::SharedPtr m_timer;
};
