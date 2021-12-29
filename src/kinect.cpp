#include "wa3li/kinect.h"

#include <algorithm>

#include "depth2scan/depth2scan.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"

KinectNode *KinectNode::m_kinect = nullptr;

bool KinectNode::imu_callback()
{
    freenect_raw_tilt_state *state = freenect_get_tilt_state(m_f_dev);
    if (!state)
    {
        RCLCPP_ERROR(get_logger(), "Could not retrieve raw imu data");
        return false;
    }

    double dx, dy, dz;
    freenect_get_mks_accel(state, &dx, &dy, &dz);

    sensor_msgs::msg::Imu imu_message;
    imu_message.header.frame_id = "kinect";
    imu_message.header.stamp = now();
    imu_message.linear_acceleration.x = dx;
    imu_message.linear_acceleration.y = dy;
    imu_message.linear_acceleration.z = dz;

    m_imu_publisher->publish(imu_message);

    return true;
}

void rgb_callback_redirect(freenect_device *, void *data, uint32_t)
{
    KinectNode *kinect = KinectNode::get_kinect();
    if (!kinect)
    {
        return;
    }
    kinect->rgb_callback(data);
}

void depth_callback_redirect(freenect_device *, void *data, uint32_t)
{
    KinectNode *kinect = KinectNode::get_kinect();
    if (!kinect)
    {
        return;
    }
    kinect->depth_callback(data);
}

sensor_msgs::msg::Image prepare_image_message(const cv::Mat &img, const std::string &encoding, unsigned pixel_size)
{
    sensor_msgs::msg::Image image_message;
    image_message.header.frame_id = "kinect";
    image_message.header.stamp = KinectNode::get_kinect()->now();
    image_message.height = img.rows;
    image_message.width = img.cols;
    image_message.encoding = encoding;
    image_message.step = img.cols * pixel_size;
    image_message.data.resize(image_message.step * img.rows);
    std::copy(img.data, img.data + image_message.data.size(), image_message.data.begin());

    return image_message;
}

void KinectNode::rgb_callback(void *data)
{
    cv::Mat frame = cv::Mat(depth2scan::limits::DEPTH_HEIGHT,
                            depth2scan::limits::DEPTH_WIDTH,
                            CV_8UC3,
                            reinterpret_cast<unsigned char *>(data));
    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    const sensor_msgs::msg::Image rgb_image =
        prepare_image_message(frame, sensor_msgs::image_encodings::TYPE_8UC3, sizeof(unsigned char) * 3);
    m_rgb_publisher->publish(rgb_image);
}

void KinectNode::depth_callback(void *data)
{
    cv::Mat frame = cv::Mat(depth2scan::limits::DEPTH_HEIGHT,
                            depth2scan::limits::DEPTH_WIDTH,
                            CV_16UC1,
                            reinterpret_cast<unsigned char *>(data));
    frame.setTo(0, frame == FREENECT_DEPTH_RAW_NO_VALUE);
    cv::Mat depth;
    frame.convertTo(depth, CV_64F);
    depth = depth * (depth2scan::limits::MAX_DIST / FREENECT_DEPTH_RAW_MAX_VALUE);

    const sensor_msgs::msg::Image depth_message =
        prepare_image_message(depth, sensor_msgs::image_encodings::TYPE_64FC1, sizeof(double));
    m_depth_publisher->publish(depth_message);

    // FIXME: Use param tilt and height
    const std::vector<double> scans = depth2scan::depth2scan(depth, 0, 0.34, nullptr);

    sensor_msgs::msg::LaserScan laser_scan_message;
    laser_scan_message.header = depth_message.header;
    laser_scan_message.angle_min = 0;
    laser_scan_message.angle_max = DEG2RAD(depth2scan::limits::HORIZONTAL_FOV);
    laser_scan_message.angle_increment = DEG2RAD(depth2scan::limits::HORIZONTAL_FOV / depth2scan::limits::DEPTH_WIDTH);
    laser_scan_message.time_increment = 0;
    laser_scan_message.scan_time = 1.0 / 30;  // FIXME: Use real values
    laser_scan_message.range_min = depth2scan::limits::MIN_DIST;
    laser_scan_message.range_max = depth2scan::limits::MAX_DIST;
    laser_scan_message.ranges.reserve(scans.size());
    for (double dist : scans)
    {
        laser_scan_message.ranges.push_back(dist);
    }
    m_laser_scan_publisher->publish(laser_scan_message);
}

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

    const int number_of_devices = freenect_num_devices(f_ctx);
    if (number_of_devices < 1)
    {
        RCLCPP_ERROR(logger, "Could not find a device or open one");
        freenect_shutdown(f_ctx);
        return nullptr;
    }

    freenect_device *f_dev;
    if (freenect_open_device(f_ctx, &f_dev, 0) < 0)
    {
        RCLCPP_ERROR(logger, "Could not open device 0");
        freenect_shutdown(f_ctx);
        return nullptr;
    }

    const int depth_res =
        freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
    const int rgb_res =
        freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
    if (depth_res < 0 || rgb_res < 0)
    {
        RCLCPP_ERROR(logger, "Could not set resolution");
        freenect_close_device(f_dev);
        freenect_shutdown(f_ctx);
        return nullptr;
    }

    freenect_set_depth_callback(f_dev, depth_callback_redirect);
    freenect_set_video_callback(f_dev, rgb_callback_redirect);

    if (freenect_start_depth(f_dev) < 0 || freenect_start_video(f_dev) < 0)
    {
        RCLCPP_ERROR(logger, "Could not start video streams");
        freenect_close_device(f_dev);
        freenect_shutdown(f_ctx);
        return nullptr;
    }

    m_kinect = new KinectNode(f_ctx, f_dev);

    return m_kinect;
}

void KinectNode::timer_callback()
{
    const int status = freenect_process_events(m_f_ctx);
    if (status < 0)
    {
        RCLCPP_ERROR(get_logger(), "An error occured with the kinect device");
        exit(-1);
    }

    if (!imu_callback())
    {
        RCLCPP_ERROR(get_logger(), "Could not process imu data");
        exit(-1);
    }
}

KinectNode::KinectNode(freenect_context *const f_ctx, freenect_device *const f_dev)
    : Node("kinect_node"), m_f_ctx(f_ctx), m_f_dev(f_dev)
{
    using namespace std::chrono_literals;

    m_laser_scan_publisher = create_publisher<sensor_msgs::msg::LaserScan>("kinect/laser_scan", 10);
    m_imu_publisher = create_publisher<sensor_msgs::msg::Imu>("kinect/imu", 10);
    m_depth_publisher = create_publisher<sensor_msgs::msg::Image>("kinect/depth", 10);
    m_rgb_publisher = create_publisher<sensor_msgs::msg::Image>("kinect/rgb", 10);

    m_timer = create_wall_timer(10ms, std::bind(&KinectNode::timer_callback, this));
}

KinectNode::~KinectNode()
{
    freenect_stop_depth(m_f_dev);
    freenect_stop_video(m_f_dev);
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
