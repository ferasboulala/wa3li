#include "wa3li/kobuki.h"

#include <cmath>
#include <limits>
#include <tuple>

#define DEG2RAD(x) (x / 180.0 * M_PI)

// TODO: stamps should as much as possible use the time the data was acquired.
// docking ir
// inertial data

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

void KobukiNode::publisher_odometry_data(const kobuki::BasicData &basic_data)
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
    transform.rotation.z =
        angular_displacement(left_angular_displacement, right_angular_displacement);
    m_transform_publisher->publish(transform);

    if (m_prev_timestamp_ms)
    {
        const double dt = basic_data.timestamp_ms - *m_prev_timestamp_ms;
        if (dt)
        {
            geometry_msgs::msg::Twist twist;
            twist.linear.x = transform.translation.x / dt * 1000;
            twist.angular.z = transform.rotation.z / dt * 1000;
            m_twist_publisher->publish(twist);
        }
    }

    m_left_encoder = basic_data.left_data.encoder;
    m_right_encoder = basic_data.right_data.encoder;
    m_prev_timestamp_ms = basic_data.timestamp_ms;
}

void KobukiNode::publish_general_sensor_data(const kobuki::BasicData &basic_data)
{
    wa3li_protocol::msg::KobukiBasicData basic_data_message;
    basic_data_message.left.bumped = basic_data.left_data.bumped;
    basic_data_message.left.cliff_sensed = basic_data.left_data.cliff_sensed;
    basic_data_message.left.wheel_dropped = basic_data.left_data.wheel_dropped;
    basic_data_message.left.overcurrent = basic_data.left_data.overcurrent;

    basic_data_message.right.bumped = basic_data.right_data.bumped;
    basic_data_message.right.cliff_sensed = basic_data.right_data.cliff_sensed;
    basic_data_message.right.wheel_dropped = basic_data.right_data.wheel_dropped;
    basic_data_message.right.overcurrent = basic_data.right_data.overcurrent;

    basic_data_message.center.bumped = basic_data.center_data.bumped;
    basic_data_message.center.cliff_sensed = basic_data.center_data.cliff_sensed;
    basic_data_message.center.wheel_dropped = false;
    basic_data_message.center.overcurrent = false;

    m_kobuki_basic_data_publisher->publish(basic_data_message);
}

void KobukiNode::publish_battery_data(const kobuki::BasicData &basic_data)
{
    sensor_msgs::msg::BatteryState battery_state;
    battery_state.header.frame_id = "kobuki";
    battery_state.header.stamp = now();
    battery_state.voltage = basic_data.battery_voltage;
    battery_state.power_supply_status =
        basic_data.is_charging ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING
                               : sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    battery_state.power_supply_technology =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

    const float NaN = std::numeric_limits<float>::quiet_NaN();
    battery_state.temperature = NaN;
    battery_state.current = NaN;
    battery_state.charge = NaN;
    battery_state.capacity = NaN;
    battery_state.design_capacity = NaN;
    battery_state.percentage = NaN;

    m_battery_state_publisher->publish(battery_state);
}

void KobukiNode::publish_cliff_data(const kobuki::CliffData &cliff_data)
{
    sensor_msgs::msg::LaserEcho echo;

    echo.echoes.resize(3);
    // TODO: Use https://global.sharp/products/device/lineup/data/pdf/datasheet/gp2y0a51sk_e.pdf
    // page 4 instead of publishing intensities.
    echo.echoes[0] = cliff_data.voltage_left;
    echo.echoes[1] = cliff_data.voltage_right;
    echo.echoes[2] = cliff_data.voltage_center;

    m_cliff_publisher->publish(echo);
}

void KobukiNode::publish_current(const kobuki::Current &current)
{
    wa3li_protocol::msg::KobukiCurrent current_message;

    current_message.left = current.current_left;
    current_message.right = current.current_left;

    m_kobuki_current_publisher->publish(current_message);
}

void KobukiNode::publish_gpi(const kobuki::GeneralPurposeInput &gpi)
{
    wa3li_protocol::msg::KobukiGpi gpi_message;

    gpi_message.digital_inputs.resize(4);
    gpi_message.digital_inputs[0] = gpi.digital_inputs[0];
    gpi_message.digital_inputs[1] = gpi.digital_inputs[1];
    gpi_message.digital_inputs[2] = gpi.digital_inputs[2];
    gpi_message.digital_inputs[3] = gpi.digital_inputs[3];
    gpi_message.voltages.resize(4);
    gpi_message.voltages[0] = gpi.voltage_0;
    gpi_message.voltages[1] = gpi.voltage_1;
    gpi_message.voltages[2] = gpi.voltage_2;
    gpi_message.voltages[3] = gpi.voltage_3;

    m_kobuki_gpi_publisher->publish(gpi_message);
}

void KobukiNode::timer_callback()
{
    kobuki::BasicData basic_data;
    kobuki::CliffData cliff_data;
    kobuki::Current current;
    kobuki::GyroData gyro_data;
    kobuki::GeneralPurposeInput gpi;

    if (m_driver->get_basic_data(basic_data, false))
    {
        publisher_odometry_data(basic_data);
        publish_general_sensor_data(basic_data);
        publish_battery_data(basic_data);
    }

    if (m_driver->get_cliff_data(cliff_data, false))
    {
        publish_cliff_data(cliff_data);
    }

    if (m_driver->get_current(current, false))
    {
        publish_current(current);
    }

    if (m_driver->get_gyro_data(gyro_data, false))
    {
        publish_imu(gyro_data);
    }

    if (m_driver->get_gpi(gpi, false))
    {
        publish_gpi(gpi);
    }
}

void KobukiNode::twist_command_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    m_driver->set_motion_twist(msg->linear.x, msg->angular.z);
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
    using std::placeholders::_1;

    m_twist_subscriber = create_subscription<geometry_msgs::msg::Twist>(
        "kobuki/cmd", 10, std::bind(&KobukiNode::twist_command_callback, this, _1));

    m_imu_publisher = create_publisher<sensor_msgs::msg::Imu>("kobuki/imu", 10);
    m_transform_publisher = create_publisher<geometry_msgs::msg::Transform>("kobuki/transform", 10);
    m_twist_publisher = create_publisher<geometry_msgs::msg::Twist>("kobuki/twist", 10);
    m_battery_state_publisher =
        create_publisher<sensor_msgs::msg::BatteryState>("kobuki/battery", 10);
    m_kobuki_basic_data_publisher =
        create_publisher<wa3li_protocol::msg::KobukiBasicData>("kobuki/sensors", 10);
    m_cliff_publisher = create_publisher<sensor_msgs::msg::LaserEcho>("kobuki/cliff", 10);
    m_kobuki_current_publisher =
        create_publisher<wa3li_protocol::msg::KobukiCurrent>("kobuki/current", 10);
    m_kobuki_gpi_publisher = create_publisher<wa3li_protocol::msg::KobukiGpi>("kobuki/gpi", 10);

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
