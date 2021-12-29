#include "wa3li/kobuki.h"

KobukiNode* KobukiNode::m_kobuki = nullptr;

void KobukiNode::timer_callback()
{
}

KobukiNode* KobukiNode::create()
{
    kobuki::Kobuki *driver = kobuki::Kobuki::create();
    if (!driver) {
        auto logger = rclcpp::get_logger("kinect_node");
        RCLCPP_ERROR(logger, "Could not create kobuki driver");
        return nullptr;
    }

    m_kobuki = new KobukiNode(driver);

    return m_kobuki;
}

KobukiNode::KobukiNode(kobuki::Kobuki *const driver) : Node("kobuki_node"), m_driver(driver)
{
    using namespace std::chrono_literals;

    m_imu_publisher = create_publisher<sensor_msgs::msg::Imu>("kobuki/imu", 10);
    m_twist_publisher = create_publisher<geometry_msgs::msg::Twist>("kobuki/twist", 10);

    m_timer = create_wall_timer(20ms, std::bind(&KobukiNode::timer_callback, this));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    KobukiNode *kobuki_node = KobukiNode::create();
    if (kobuki_node)
    {
        rclcpp::spin(std::shared_ptr<KobukiNode>(kobuki_node));
    }
    rclcpp::shutdown();

    return 0;
}
