#include "wa3li/nav.h"

#include <limits>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "slam/util.h"
#include "tf2/LinearMath/Quaternion.h"

#define MAP_RESOLUTION 0.01  // cm

// TODO: Make mcl's canvas size a parameter

void SLAMNode::publish_queued_transforms()
{
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = "map";
    for (const auto &tf : m_transform_queue)
    {
        slam::Odometry odom = {tf.rotation.z / 2, tf.translation.x, tf.rotation.z / 2};
        odom.translation /= MAP_RESOLUTION;
        m_mcl->predict(odom, {0.001, 0.001, 0.001, 0.001});

        const slam::Pose estimated_pose = slam::average_pose(m_mcl->get_particles());
        geometry_msgs::msg::TransformStamped transform_message;
        transform_message.header = header;
        transform_message.child_frame_id = "robot";
        // TODO : Clean up
        transform_message.transform.translation.x = estimated_pose.x * MAP_RESOLUTION - 5;
        transform_message.transform.translation.y = estimated_pose.y * MAP_RESOLUTION - 5;

        tf2::Quaternion q;
        q.setRPY(0, 0, estimated_pose.theta);
        q.normalize();
        transform_message.transform.rotation.x = q.x();
        transform_message.transform.rotation.y = q.y();
        transform_message.transform.rotation.z = q.z();
        transform_message.transform.rotation.w = q.w();

        m_tf_publisher->sendTransform(transform_message);
    }

    m_transform_queue.clear();
}

void SLAMNode::transform_command_callback(const geometry_msgs::msg::Transform::SharedPtr msg)
{
    std::unique_lock<std::mutex> lock(m_transform_mutex);
    m_transform_queue.push_back(*msg);
    lock.unlock();

    if (!m_write_mutex.try_lock())
    {
        return;
    }

    publish_queued_transforms();

    m_write_mutex.unlock();
}

static void draw_particle(
    cv::Mat &img, const slam::Pose &pose, cv::Scalar color, int size, bool filled = false)
{
    const auto coord = slam::pose_to_image_coordinates(img, pose);
    int i, j;
    std::tie(i, j) = coord;
    cv::circle(img, {j, i}, size, color, filled ? cv::FILLED : 0);

    const double x = pose.x + 10 * size * std::cos(pose.theta);
    const double y = pose.y + 10 * size * std::sin(pose.theta);
    const auto coord_ = slam::pose_to_image_coordinates(img, {x, y, 0});
    int i_, j_;
    std::tie(i_, j_) = coord_;
    cv::line(img, {j, i}, {j_, i_}, color);
}

#include "slam/colors.h"
void SLAMNode::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> write_guard(m_write_mutex);

    // FIXME: This should be in the laser scan message
    // or sent as a topic
    constexpr double STDDEV = 1;  // cell domain
    const double z_max = msg->range_max / MAP_RESOLUTION;
    std::vector<std::tuple<double, double>> scans;
    for (unsigned i = 0; i < msg->ranges.size(); ++i)
    {
        if (msg->ranges[i] == msg->range_max) continue;
        const double angle = i * msg->angle_increment + msg->angle_min;
        scans.push_back({angle, msg->ranges[i] / MAP_RESOLUTION});
    }

    // TODO: Make this a parameter
    // Use utility function for meters to cells
    // Make that a wrapper around slam::MCL that accepts meters
    constexpr double KINECT_Y_OFFSET = -0.065 / MAP_RESOLUTION;
    constexpr double KINECT_X_OFFSET = 0.065 / MAP_RESOLUTION;
    constexpr slam::Pose sensor_offset = {KINECT_X_OFFSET, KINECT_Y_OFFSET, 0};
    m_mcl->update(scans, STDDEV, z_max, sensor_offset);
    const cv::Mat &best_map = m_mcl->get_particles().front().map;
    cv::Mat frame = best_map.clone();
    for (const slam::Particle &particle : m_mcl->get_particles())
    {
        draw_particle(frame, particle.pose, GREEN, 5, true);
    }
    cv::imshow("map", frame);
    cv::waitKey(10);

    // nav_msgs::msg::OccupancyGrid grid_message;
    // grid_message.header.frame_id = "nav";
    // grid_message.header.stamp = now();
    // grid_message.info.map_load_time = now();
    // grid_message.info.resolution = MAP_RESOLUTION;
    // grid_message.info.width = best_map.cols;
    // grid_message.info.height = best_map.rows;

    // cv::Mat int8_map = best_map * 100;
    // int8_map.convertTo(int8_map, CV_8UC1);
    // int8_map = 100 - int8_map;
    // int8_map.setTo(-1, int8_map == 50);  // TODO: Do it at the slam:: level
    // grid_message.data.resize(best_map.total());
    // std::copy(best_map.data, best_map.data + best_map.total(), grid_message.data.begin());

    // m_grid_publisher->publish(grid_message);

    std::lock_guard<std::mutex> queue_guard(m_transform_mutex);

    publish_queued_transforms();
}

SLAMNode::SLAMNode() : Node("slam_node")
{
    using std::placeholders::_1;

    m_transform_callback_group =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_scan_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto transform_opt = rclcpp::SubscriptionOptions();
    transform_opt.callback_group = m_transform_callback_group;
    auto scan_opt = rclcpp::SubscriptionOptions();
    scan_opt.callback_group = m_scan_callback_group;

    rclcpp::QoS qos(5);
    qos.keep_all();
    qos.reliable();
    qos.durability_volatile();
    m_transform_subscriber = create_subscription<geometry_msgs::msg::Transform>(
        "kobuki/odom/transform",
        qos,
        std::bind(&SLAMNode::transform_command_callback, this, _1),
        transform_opt);

    m_scan_subscriber = create_subscription<sensor_msgs::msg::LaserScan>(
        "kinect/scan", 1, std::bind(&SLAMNode::laser_scan_callback, this, _1), scan_opt);

    m_tf_publisher = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    m_grid_publisher = create_publisher<nav_msgs::msg::OccupancyGrid>("nav/grid", 10);

    // TODO: Use params for number of particles
    m_mcl = std::make_unique<slam::MCL>(50);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto slam_node = std::make_shared<SLAMNode>();
    executor.add_node(slam_node);
    executor.spin();
    rclcpp::shutdown();

    return 0;
}
