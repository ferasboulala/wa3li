#include <memory>
#include <optional>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam/mcl.h"

class SLAMNode : public rclcpp::Node
{
public:
    SLAMNode();
    ~SLAMNode() = default;

private:
    void twist_command_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_scan_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_twist_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_pose_publisher;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr m_grid_publisher;

    std::unique_ptr<slam::MCL> m_mcl;
    std::optional<rclcpp::Time> m_last_predict_stamp;
};
