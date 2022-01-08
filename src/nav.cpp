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
// TODO: Use the appropriate QoS settings for cmd
// TODO: Stamp the Twist cmd and get rid of m_last_predict_stamp

void SLAMNode::twist_command_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    const rclcpp::Time t = now();
    if (!m_last_predict_stamp)
    {
        m_last_predict_stamp = t;
        return;
    }
    const rclcpp::Duration diff = t - *m_last_predict_stamp;
    const double dt = diff.seconds();
    slam::Odometry odom = {msg->angular.z / 2 * dt, msg->linear.x * dt, msg->angular.z / 2 * dt};
    odom.translation /= MAP_RESOLUTION;
    m_mcl->predict(odom, {0.005, 0.005, 0.05, 0.05});

    const slam::Pose estimated_pose = slam::average_pose(m_mcl->get_particles());
    geometry_msgs::msg::TransformStamped transform_message;
    transform_message.header.stamp = t;
    transform_message.header.frame_id = "map";
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
    m_last_predict_stamp = t;
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
    cv::imshow("map", best_map);
    cv::waitKey(33);

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
}

SLAMNode::SLAMNode() : Node("slam_node")
{
    using std::placeholders::_1;

    m_twist_subscriber = create_subscription<geometry_msgs::msg::Twist>(
        "kobuki/odom/twist", 10, std::bind(&SLAMNode::twist_command_callback, this, _1));
    m_scan_subscriber = create_subscription<sensor_msgs::msg::LaserScan>(
        "kinect/scan", 10, std::bind(&SLAMNode::laser_scan_callback, this, _1));

    m_tf_publisher = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    m_grid_publisher = create_publisher<nav_msgs::msg::OccupancyGrid>("nav/grid", 10);

    // TODO: Use params for number of particles
    m_mcl = std::make_unique<slam::MCL>(10);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SLAMNode>());
    rclcpp::shutdown();

    return 0;
}
