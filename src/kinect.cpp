#include "kinect.h"

#include "rclcpp/rclcpp.hpp"

KinectNode *KinectNode::m_kinect;

void depth_callback(freenect_device *, void *, uint32_t) {}

KinectNode *KinectNode::create()
{
    if (m_kinect)
    {
        return m_kinect;
    }

    auto logger = rclcpp::get_logger("kinect_node");

    freenect_context *f_ctx;
    if (freenect_init(&f_ctx, NULL) < 0)
    {
        RCLCPP_ERROR(logger, "Could not initialize freenect device");
        return nullptr;
    }

    freenect_set_log_level(f_ctx, FREENECT_LOG_ERROR);
    freenect_select_subdevices(f_ctx,
                               static_cast<freenect_device_flags>(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

    const int number_of_devices = freenect_num_devices(f_ctx);
    if (number_of_devices < 1)
    {
        RCLCPP_ERROR(logger, "Could not find a device or open one");
        freenect_shutdown(f_ctx);
        return nullptr;
    }

    RCLCPP_INFO(logger, "Number of devices found : %d", number_of_devices);

    freenect_device *f_dev;
    if (freenect_open_device(f_ctx, &f_dev, 0) < 0)
    {
        RCLCPP_ERROR(logger, "Could not open device 0");
        freenect_shutdown(f_ctx);
        return nullptr;
    }

    freenect_set_depth_callback(f_dev, depth_callback);
    const int res =
        freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
    if (res < 0)
    {
        RCLCPP_ERROR(logger, "Could not set resolution");
        freenect_close_device(f_dev);
        freenect_shutdown(f_ctx);
        return nullptr;
    }

    freenect_start_depth(f_dev);

    m_kinect = new KinectNode(f_ctx, f_dev);

    return m_kinect;
}

void KinectNode::timer_callback()
{
    const int status = freenect_process_events(m_f_ctx);
    if (status < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "An error occured with the kinect device");
        exit(-1);
    }
}

KinectNode::KinectNode(freenect_context *const f_ctx, freenect_device *const f_dev)
    : Node("kinect_node"), m_f_ctx(f_ctx), m_f_dev(f_dev)
{
    using namespace std::chrono_literals;
    m_timer = this->create_wall_timer(10ms, std::bind(&KinectNode::timer_callback, this));
}

KinectNode::~KinectNode()
{
    freenect_stop_depth(m_f_dev);
    freenect_close_device(m_f_dev);
    freenect_shutdown(m_f_ctx);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    KinectNode *kinect_node = KinectNode::create();
    if (kinect_node)
    {
        rclcpp::spin(std::shared_ptr<KinectNode>(kinect_node));
    }
    rclcpp::shutdown();

    return 0;
}
