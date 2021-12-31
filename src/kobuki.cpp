#include "wa3li/kobuki.h"

#include <cmath>
#include <limits>
#include <tuple>

#define DEG2RAD(x) (x / 180.0 * M_PI)

KobukiNode *KobukiNode::m_kobuki = nullptr;

void KobukiNode::publish_imu(const kobuki::GyroData &gyro_data)
{
    sensor_msgs::msg::Imu imu_message;
    imu_message.header.frame_id = "kobuki";
    imu_message.header.stamp = now();
    imu_message.angular_velocity.x = DEG2RAD(gyro_data.wx);
    imu_message.angular_velocity.y = DEG2RAD(gyro_data.wy);
    imu_message.angular_velocity.z = DEG2RAD(gyro_data.wz);

    m_imu_publisher->publish(imu_message);
}

static inline double translational_displacement(double left, double right)
{
    return kobuki::WHEEL_RADIUS / 2 * (left + right);
}

static inline double angular_displacement(double left, double right)
{
    return kobuki::WHEEL_RADIUS / kobuki::WHEEL_BASE * (right - left);
}

static inline std::tuple<uint16_t, int> tick_diff(unsigned short before, unsigned short after)
{
    const uint16_t diff = after - before;
    if (diff > std::numeric_limits<uint16_t>::max() / 2)
    {
        return {before - after, -1};
    }

    return {diff, 1};
}

void KobukiNode::publish_transform(const kobuki::BasicData &basic_data)
{
    geometry_msgs::msg::Transform transform;
    const auto [left_diff, left_sign] = tick_diff(
        m_left_encoder.value_or(basic_data.left_data.encoder), basic_data.left_data.encoder);
    const auto [right_diff, right_sign] = tick_diff(
        m_right_encoder.value_or(basic_data.right_data.encoder), basic_data.right_data.encoder);
    const double left_angular_displacement =
        kobuki::Kobuki::ticks_to_radians(left_diff) * left_sign;
    const double right_angular_displacement =
        kobuki::Kobuki::ticks_to_radians(right_diff) * right_sign;
    // To be interpreted as just translational displacement
    transform.translation.x =
        translational_displacement(left_angular_displacement, right_angular_displacement);
    transform.translation.y = 0;
    transform.translation.z = 0;
    transform.rotation.x = 0;
    transform.rotation.y = 0;
    transform.rotation.z =
        angular_displacement(left_angular_displacement, right_angular_displacement);

    m_transform_publisher->publish(transform);

    m_left_encoder = basic_data.left_data.encoder;
    m_right_encoder = basic_data.right_data.encoder;
}

void KobukiNode::timer_callback()
{
    kobuki::BasicData basic_data;
    kobuki::DockingIR docking_ir;
    kobuki::InertialData inertial_data;
    kobuki::CliffData cliff_data;
    kobuki::Current current;
    kobuki::GyroData gyro_data;
    kobuki::GeneralPurposeInput gpi;
    // TODO: Make this a service
    // kobuki::PID pid;

    if (m_driver->get_basic_data(basic_data, false))
    {
        publish_transform(basic_data);
    }

    if (m_driver->get_docking_ir(docking_ir, false))
    {
    }

    if (m_driver->get_inertial_data(inertial_data, false))
    {
    }

    if (m_driver->get_cliff_data(cliff_data, false))
    {
    }

    if (m_driver->get_current(current, false))
    {
    }

    if (m_driver->get_gyro_data(gyro_data, false))
    {
        publish_imu(gyro_data);
    }

    if (m_driver->get_gpi(gpi, false))
    {
    }
}

KobukiNode *KobukiNode::create()
{
    kobuki::Kobuki *driver = kobuki::Kobuki::create();
    if (!driver)
    {
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
    m_transform_publisher = create_publisher<geometry_msgs::msg::Transform>("kobuki/transform", 10);

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
