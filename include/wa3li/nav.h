#include <memory>

#include "geometry_msgs/msg/pose.hpp"
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
    void transform_command_callback(const geometry_msgs::msg::Transform::SharedPtr msg);
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scan_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Transform>::SharedPtr m_twist_subscriber;

    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_publisher;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_grid_publisher;

    std::unique_ptr<slam::MCL> m_mcl;
};
