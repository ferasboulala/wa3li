#include "wa3li/nav.h"

#include <limits>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <vector>

#include "slam/util.h"
#include "tf2/LinearMath/Quaternion.h"

#define MAP_RESOLUTION 0.01  // cm

// TODO: Make mcl's canvas size a parameter
// TODO: Use the appropriate QoS settings for cmd
// TODO: Stamp the Twist cmd and get rid of m_last_predict_stamp

void SLAMNode::twist_command_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    const rclcpp::Time t = now();
    if (m_last_predict_stamp)
    {
        const rclcpp::Duration diff = t - *m_last_predict_stamp;
        const double dt = diff.seconds();
        slam::Odometry odom = {
            msg->angular.z / 2 * dt, msg->linear.x * dt, msg->angular.z / 2 * dt};
        odom.translation /= MAP_RESOLUTION;
        m_mcl->predict(odom, {0.0005, 0.0005, 0.01, 0.01});
    }
    m_last_predict_stamp = t;

    const slam::Pose estimated_pose = slam::average_pose(m_mcl->get_particles());
    geometry_msgs::msg::Pose pose_message;
    pose_message.position.x = estimated_pose.x;
    pose_message.position.y = estimated_pose.y;

    tf2::Quaternion q;
    q.setRPY(0, 0, estimated_pose.theta);
    pose_message.orientation.x = q.x();
    pose_message.orientation.y = q.y();
    pose_message.orientation.z = q.z();
    pose_message.orientation.w = q.w();

    m_pose_publisher->publish(pose_message);
}

void SLAMNode::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // FIXME: This should be in the laser scan message
    // or sent as a topic
    constexpr double STDDEV = 1;  // cell domain
    const double z_max = msg->range_max / MAP_RESOLUTION;
    std::vector<std::tuple<double, double>> scans;
    for (unsigned i = 0; i < msg->ranges.size(); ++i)
    {
        // TODO: Compare to z_max instead
        // change that in depth2scan
        // if (msg->ranges[i] == std::numeric_limits<double>::max())
        //{
        //    continue;
        //}

        const double angle = i * msg->angle_increment + msg->angle_min;
        scans.push_back({angle, msg->ranges[i] / MAP_RESOLUTION});
    }

    m_mcl->update(scans, STDDEV, z_max);
    const cv::Mat &best_map = m_mcl->get_particles().front().map;

    nav_msgs::msg::OccupancyGrid grid_message;
    grid_message.header.frame_id = "nav";
    grid_message.header.stamp = now();
    grid_message.info.map_load_time = now();
    grid_message.info.resolution = MAP_RESOLUTION;
    grid_message.info.width = best_map.cols;
    grid_message.info.height = best_map.rows;

    cv::Mat int8_map = best_map * 100;
    int8_map.convertTo(int8_map, CV_8UC1);
    int8_map = 100 - int8_map;
    int8_map.setTo(255, int8_map == 50);  // TODO: Do it at the slam:: level
    cv::imshow("foo", int8_map);
    cv::waitKey(33);
    grid_message.data.resize(best_map.total());
    std::copy(best_map.data, best_map.data + best_map.total(), grid_message.data.begin());

    m_grid_publisher->publish(grid_message);
}

SLAMNode::SLAMNode() : Node("kinect_node")
{
    using std::placeholders::_1;

    m_twist_subscriber = create_subscription<geometry_msgs::msg::Twist>(
        "nav/cmd", 10, std::bind(&SLAMNode::twist_command_callback, this, _1));
    m_scan_subscriber = create_subscription<sensor_msgs::msg::LaserScan>(
        "nav/scan", 10, std::bind(&SLAMNode::laser_scan_callback, this, _1));

    m_pose_publisher = create_publisher<geometry_msgs::msg::Pose>("nav/pose", 10);
    m_grid_publisher = create_publisher<nav_msgs::msg::OccupancyGrid>("nav/grid", 10);

    // TODO: Use params for number of particles
    m_mcl = std::make_unique<slam::MCL>(25);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SLAMNode>());
    rclcpp::shutdown();

    return 0;
}
