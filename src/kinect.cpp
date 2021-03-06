#include "wa3li/kinect.h"

#include <algorithm>

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

static void rgb_callback_redirect(freenect_device *, void *data, uint32_t)
{
    KinectNode *kinect = KinectNode::get_kinect();
    if (!kinect)
    {
        return;
    }
    kinect->rgb_callback(data);
}

static void depth_callback_redirect(freenect_device *, void *data, uint32_t)
{
    KinectNode *kinect = KinectNode::get_kinect();
    if (!kinect)
    {
        return;
    }
    kinect->depth_callback(data);
}

sensor_msgs::msg::Image prepare_image_message(const cv::Mat &img,
                                              const std::string &encoding,
                                              unsigned pixel_size)
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
    constexpr unsigned KINECT_RGB_HEIGHT = 480;
    constexpr unsigned KINECT_RGB_WIDTH = 640;

    cv::Mat frame = cv::Mat(
        KINECT_RGB_HEIGHT, KINECT_RGB_WIDTH, CV_8UC3, reinterpret_cast<unsigned char *>(data));
    cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
    const sensor_msgs::msg::Image rgb_image = prepare_image_message(
        frame, sensor_msgs::image_encodings::TYPE_8UC3, sizeof(unsigned char) * 3);
    m_rgb_publisher->publish(rgb_image);
}

void KinectNode::depth_callback(void *data)
{
    cv::Mat frame = cv::Mat(
        m_camera_info.rows, m_camera_info.cols, CV_16UC1, reinterpret_cast<unsigned char *>(data));
    cv::Mat depth;
    frame.convertTo(depth, CV_32F, 0.001);

    const sensor_msgs::msg::Image depth_message =
        prepare_image_message(depth, sensor_msgs::image_encodings::TYPE_32FC1, sizeof(float));
    m_depth_publisher->publish(depth_message);

    sensor_msgs::msg::CameraInfo info_message;
    info_message.header = depth_message.header;
    info_message.height = m_camera_info.rows;
    info_message.width = m_camera_info.cols;
    info_message.distortion_model = "plumb_bob";
    info_message.d.resize(5);
    info_message.k[0] = 570;
    info_message.k[2] = 314;
    info_message.k[4] = 570;
    info_message.k[5] = 235;
    info_message.k[8] = 1.0;
    info_message.r[0] = 1.0;
    info_message.r[4] = 1.0;
    info_message.r[8] = 1.0;
    info_message.p[0] = 570;
    info_message.p[2] = 314;
    info_message.p[5] = 570;
    info_message.p[6] = 235;
    info_message.p[10] = 1.0;
    m_camera_info_publisher->publish(info_message);

    const std::vector<std::tuple<double, double>> scans =
        m_converter->convert<float>(depth, DEG2RAD(m_angle), m_height);

    sensor_msgs::msg::LaserScan laser_scan_message;
    laser_scan_message.header = depth_message.header;
    laser_scan_message.angle_min = std::get<0>(scans.front());
    laser_scan_message.angle_max = std::get<0>(scans.back());
    laser_scan_message.angle_increment = DEG2RAD(m_camera_info.hfov) / m_camera_info.cols;
    laser_scan_message.time_increment = 0;
    laser_scan_message.scan_time = 1.0 / 30;
    laser_scan_message.range_min = m_camera_info.min_dist;
    laser_scan_message.range_max = m_camera_info.max_dist;
    laser_scan_message.ranges.reserve(scans.size());
    for (auto [angle, dist] : scans)
    {
        if (dist == std::numeric_limits<double>::max())
        {
            dist = m_camera_info.max_dist;
        }
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

    const int depth_res = freenect_set_depth_mode(
        f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_MM));
    const int rgb_res = freenect_set_video_mode(
        f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB));
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

void KinectNode::parameters_timer_callback()
{
    get_parameter("height", m_height);
    get_parameter("angle", m_angle);
}

void KinectNode::topics_timer_callback()
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

void KinectNode::get_tilt_callback(const std::shared_ptr<wa3li_protocol::srv::GetTilt::Request>,
                                   std::shared_ptr<wa3li_protocol::srv::GetTilt::Response> response)
{
    freenect_raw_tilt_state *state = freenect_get_tilt_state(m_f_dev);
    response->tilt = DEG2RAD(freenect_get_tilt_degs(state));
}

static void get_tilt_callback_redirect(
    const std::shared_ptr<wa3li_protocol::srv::GetTilt::Request> request,
    std::shared_ptr<wa3li_protocol::srv::GetTilt::Response> response)
{
    KinectNode::get_kinect()->get_tilt_callback(request, response);
}

void KinectNode::set_tilt_callback(
    const std::shared_ptr<wa3li_protocol::srv::SetTilt::Request> request,
    std::shared_ptr<wa3li_protocol::srv::SetTilt::Response>)
{
    freenect_set_tilt_degs(m_f_dev, request->tilt);
    // TODO: If necessary, use an action instead of a service
    // while (true)
    //{
    //    freenect_raw_tilt_state *state = freenect_get_tilt_state(m_f_dev);
    //    if (freenect_get_tilt_status(state) == TILT_STATUS_STOPPED)
    //        break;
    //}
}

static void set_tilt_callback_redirect(
    const std::shared_ptr<wa3li_protocol::srv::SetTilt::Request> request,
    std::shared_ptr<wa3li_protocol::srv::SetTilt::Response> response)
{
    KinectNode::get_kinect()->set_tilt_callback(request, response);
}

void KinectNode::led_callback(const std::shared_ptr<wa3li_protocol::srv::Led::Request> request,
                              std::shared_ptr<wa3li_protocol::srv::Led::Response>)
{
    static const std::unordered_map<std::string, freenect_led_options> led_options = {
        {"off", LED_OFF},
        {"green", LED_GREEN},
        {"red", LED_RED},
        {"yellow", LED_YELLOW},
        {"blink_green", LED_BLINK_GREEN},
        {"blink_red_yellow", LED_BLINK_RED_YELLOW}};

    const auto it = led_options.find(request->color);
    if (it == led_options.end())
    {
        RCLCPP_ERROR(get_logger(), "Unrecognized led color %s", request->color.c_str());
        return;
    }
    freenect_set_led(m_f_dev, it->second);
}

static void led_callback_redirect(const std::shared_ptr<wa3li_protocol::srv::Led::Request> request,
                                  std::shared_ptr<wa3li_protocol::srv::Led::Response> response)
{
    KinectNode::get_kinect()->led_callback(request, response);
}

KinectNode::KinectNode(freenect_context *const f_ctx, freenect_device *const f_dev)
    : Node("kinect_node"), m_f_ctx(f_ctx), m_f_dev(f_dev), m_height(0.34), m_angle(0)
{
    using namespace std::chrono_literals;

    m_imu_publisher =
        create_publisher<sensor_msgs::msg::Imu>("kinect/imu", rclcpp::SensorDataQoS());
    m_laser_scan_publisher = create_publisher<sensor_msgs::msg::LaserScan>("kinect/scan", 1);
    m_depth_publisher = create_publisher<sensor_msgs::msg::Image>("kinect/depth", 1);
    m_camera_info_publisher =
        create_publisher<sensor_msgs::msg::CameraInfo>("kinect/camera_info", 1);
    m_rgb_publisher =
        create_publisher<sensor_msgs::msg::Image>("kinect/rgb", rclcpp::SensorDataQoS());

    // Does not support std::bind ?. ROS2 examples do not use it too.
    m_get_tilt_service = create_service<wa3li_protocol::srv::GetTilt>("kinect/get_tilt",
                                                                      &get_tilt_callback_redirect);
    m_set_tilt_service = create_service<wa3li_protocol::srv::SetTilt>("kinect/set_tilt",
                                                                      &set_tilt_callback_redirect);
    m_led_service = create_service<wa3li_protocol::srv::Led>("kinect/led", &led_callback_redirect);

    declare_parameter<double>("height", 0.34);
    declare_parameter<double>("angle", 0);

    m_camera_info.rows = 480;
    m_camera_info.cols = 640;
    m_camera_info.vfov = 46.6;
    m_camera_info.hfov = 58.5;
    m_camera_info.min_dist = 0.4;
    m_camera_info.max_dist = 5;
    m_converter = std::make_unique<depth2scan::Converter>(m_camera_info);

    m_topics_timer = create_wall_timer(10ms, std::bind(&KinectNode::topics_timer_callback, this));
    m_parameters_timer =
        create_wall_timer(1s, std::bind(&KinectNode::parameters_timer_callback, this));
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
    if (!kinect_node)
    {
        RCLCPP_ERROR(rclcpp::get_logger("kinect_node"), "Could not create kinect node");
        return -1;
    }

    rclcpp::spin(std::shared_ptr<KinectNode>(kinect_node));
    rclcpp::shutdown();

    return 0;
}
