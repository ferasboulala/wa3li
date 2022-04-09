#include <array>
#include <memory>
#include <mutex>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam/mcl.h"
#include "tf2_ros/transform_broadcaster.h"

class SLAMNode : public rclcpp::Node
{
public:
    SLAMNode();
    ~SLAMNode() = default;

private:
    slam::Pose normalize(const slam::Pose &pose) const;
    void publish_queued_transforms();
    void transform_command_callback(const geometry_msgs::msg::Transform::SharedPtr msg);
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::CallbackGroup::SharedPtr m_scan_callback_group;
    rclcpp::CallbackGroup::SharedPtr m_transform_callback_group;

    rclcpp::Subscription<geometry_msgs::msg::Transform>::SharedPtr m_transform_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scan_subscriber;

    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_publisher;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_grid_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_pose_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_pose_array_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovariance>::SharedPtr
        m_pose_with_covariance_publisher;

    double m_scan_stddev;
    double m_cell_resolution;
    double m_scanner_offset_x;
    double m_scanner_offset_y;
    unsigned m_every_other_ray;
    std::array<double, 4> m_motion_error;
    std::unique_ptr<slam::MCL> m_mcl;

    std::mutex m_transform_mutex;
    std::mutex m_write_mutex;

    std::vector<geometry_msgs::msg::Transform> m_transform_queue;
};
