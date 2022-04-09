#include "wa3li/nav.h"

#include <limits>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "slam/util.h"
#include "tf2/LinearMath/Quaternion.h"

static void tf2_to_msg_quaternion(const tf2::Quaternion &q, geometry_msgs::msg::Quaternion &msg)
{
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
}

// TODO: Move to slam::
slam::Pose SLAMNode::normalize(const slam::Pose &pose) const
{
    const slam::Pose starting_pose = m_mcl->starting_pose();

    slam::Pose normalized_pose;
    normalized_pose.x = (pose.x - starting_pose.x) * m_cell_resolution;
    normalized_pose.y = (pose.y - starting_pose.y) * m_cell_resolution;
    normalized_pose.theta = pose.theta;

    return normalized_pose;
}

void SLAMNode::publish_queued_transforms()
{
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = "map";

    geometry_msgs::msg::PoseArray pose_array_message;
    pose_array_message.header = header;

    for (const auto &tf : m_transform_queue)
    {
        // Because ROS' common interfaces do not have a translation + rotation message, the
        // convention is to send out a tf and to embed the values in the x and z values.
        slam::Odometry odom = {tf.rotation.z / 2, tf.translation.x, tf.rotation.z / 2};
        odom.translation /= m_cell_resolution;

        m_mcl->predict(odom, m_motion_error);

        const slam::Pose estimated_pose = slam::average_pose(m_mcl->get_particles());
        geometry_msgs::msg::TransformStamped transform_message;
        transform_message.header = header;
        transform_message.child_frame_id = "robot";

        const slam::Pose normalized_pose = normalize(estimated_pose);
        transform_message.transform.translation.x = normalized_pose.x;
        transform_message.transform.translation.y = normalized_pose.y;

        tf2::Quaternion q;
        // FIXME: Figure out why rviz displays angles in the opposite way
        q.setRPY(0, 0, -estimated_pose.theta);
        q.normalize();
        tf2_to_msg_quaternion(q, transform_message.transform.rotation);

        geometry_msgs::msg::Pose pose_message;
        pose_message.position.x = transform_message.transform.translation.x;
        pose_message.position.y = transform_message.transform.translation.y;
        pose_message.orientation = transform_message.transform.rotation;

        m_tf_publisher->sendTransform(transform_message);
        m_pose_publisher->publish(pose_message);
    }

    const std::vector<slam::Particle> &particles = m_mcl->get_particles();
    pose_array_message.poses.reserve(particles.size());
    for (const slam::Particle &particle : particles)
    {
        geometry_msgs::msg::Pose pose;

        const slam::Pose normalized_pose = normalize(particle.pose);
        pose.position.x = normalized_pose.x;
        pose.position.y = normalized_pose.y;

        tf2::Quaternion q;
        // FIXME: Why TF is this not negative in rviz??
        q.setRPY(0, 0, particle.pose.theta);
        q.normalize();

        tf2_to_msg_quaternion(q, pose.orientation);

        pose_array_message.poses.push_back(pose);
    }

    m_pose_array_publisher->publish(pose_array_message);
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

void SLAMNode::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> write_guard(m_write_mutex);

    const double scan_stddev_cell_domain = m_scan_stddev / m_cell_resolution;
    const double dist_max_cell_domain = msg->range_max / m_cell_resolution;

    std::vector<std::tuple<double, double>> scans_cell_domain;
    for (unsigned i = 0; i < msg->ranges.size(); ++i)
    {
        // The range max readings are discarded because, in the case of a sensor like a kinect, it
        // will not provide enough information to the map. That is, the kinect will return a maximum
        // reading whenever the ray is too close or too far in which case, the cells in the path
        // cannot be considered free or occupied.
        if (msg->ranges[i] == msg->range_max) continue;
        if (i % m_every_other_ray) continue;

        const double angle = i * msg->angle_increment + msg->angle_min;
        scans_cell_domain.emplace_back(angle, msg->ranges[i] / m_cell_resolution);
    }

    const double scanner_offset_y_cell_domain = m_scanner_offset_y / m_cell_resolution;
    const double scanner_offset_x_cell_domain = m_scanner_offset_x / m_cell_resolution;
    const slam::Pose sensor_offset = {
        scanner_offset_x_cell_domain, scanner_offset_y_cell_domain, 0};

    m_mcl->update(scans_cell_domain, scan_stddev_cell_domain, dist_max_cell_domain, sensor_offset);

    const cv::Mat &best_map = m_mcl->get_particles().front().map;

    nav_msgs::msg::OccupancyGrid grid_message;
    grid_message.header.frame_id = "map";
    grid_message.header.stamp = now();
    grid_message.info.map_load_time = grid_message.header.stamp;
    grid_message.info.resolution = m_cell_resolution;
    grid_message.info.width = best_map.cols;
    grid_message.info.height = best_map.rows;

    const slam::Pose starting_pose = m_mcl->starting_pose();
    grid_message.info.origin.position.x = -starting_pose.x * m_cell_resolution;
    grid_message.info.origin.position.y = -starting_pose.y * m_cell_resolution;

    cv::Mat display_map;
    display_map = 100 - (best_map / (static_cast<double>(255) / 100));
    cv::flip(display_map, display_map, -1);  // FIXME: Figure out why rviz displays it flipped
    grid_message.data.resize(display_map.total());
    std::copy(display_map.data, display_map.data + display_map.total(), grid_message.data.begin());
    m_grid_publisher->publish(grid_message);

    std::lock_guard<std::mutex> queue_guard(m_transform_mutex);
    publish_queued_transforms();
}

SLAMNode::SLAMNode() : Node("slam_node")
{
    using std::placeholders::_1;

    // ON THE MULTITHREADED DESIGN:
    // The kobuki will publish the odometry data at a rate of 50Hz. The kinect will publish the
    // laser scan at a rate of 30Hz. The predict() step is very fast meaning the odometry data
    // will never queue. On the other hand, the update() step can take up to 100 ms to finish
    // (depends on parameters like number of particles, number of rays and sensor maximum distance).
    // With a single threaded node, the odometry data will end up queueing. Because the ROS
    // executors iterate through the subscribed topis in round robin fashion, the node would process
    // one scan, then one odometry update, then one scan again. The second scan is up to date but
    // the odometry data is very old which causes the system to completely fail and to queue up
    // messages at a rate of 100 ms / 20 ms = 5 Hz. To circumvent this, the node uses
    // a multithreaded executor and empties the whole buffer of odometry after a laser scan message
    // was processed.

    declare_parameter<std::string>("transform_topic", "kobuki/odom/transform");
    declare_parameter<std::string>("scan_topic", "kinect/scan");
    declare_parameter<int>("n_particles", 200);
    declare_parameter<int>("every_other_ray", 10);
    // https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3304120/#:~:text=Experimental%20results%20show%20that%20the,resolution%20of%20the%20depth%20measurements.
    declare_parameter<double>("scan_stddev", 0.04);
    declare_parameter<double>("cell_resolution", 0.05);
    declare_parameter<double>("scanner_offset_x", 0.065);
    declare_parameter<double>("scanner_offset_y", -0.065);
    declare_parameter<double>("alpha_0", 0.0001);
    declare_parameter<double>("alpha_1", 0.0001);
    declare_parameter<double>("alpha_2", 0.001);
    declare_parameter<double>("alpha_3", 0.001);

    std::string transform_topic;
    std::string scan_topic;
    unsigned n_particles;

    get_parameter("transform_topic", transform_topic);
    get_parameter("scan_topic", scan_topic);
    get_parameter("n_particles", n_particles);
    get_parameter("every_other_ray", m_every_other_ray);
    get_parameter("scan_stddev", m_scan_stddev);
    get_parameter("cell_resolution", m_cell_resolution);
    get_parameter("scanner_offset_x", m_scanner_offset_x);
    get_parameter("scanner_offset_y", m_scanner_offset_y);
    get_parameter("alpha_0", m_motion_error[0]);
    get_parameter("alpha_1", m_motion_error[1]);
    get_parameter("alpha_2", m_motion_error[2]);
    get_parameter("alpha_3", m_motion_error[3]);

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
        transform_topic,
        qos,
        std::bind(&SLAMNode::transform_command_callback, this, _1),
        transform_opt);

    // The algorithm needs the latest laser scan only because in the event that it processes them
    // not as fast as the src publishes them, queuing them up is pointless.
    m_scan_subscriber = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, 1, std::bind(&SLAMNode::laser_scan_callback, this, _1), scan_opt);

    m_tf_publisher = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = "map";

    geometry_msgs::msg::TransformStamped transform_message;
    transform_message.header.stamp = now();
    transform_message.header.frame_id = "map";
    transform_message.child_frame_id = "kinect";
    transform_message.transform.translation.x = m_scanner_offset_x;
    transform_message.transform.translation.y = m_scanner_offset_y;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    q.normalize();
    tf2_to_msg_quaternion(q, transform_message.transform.rotation);
    m_tf_publisher->sendTransform(transform_message);

    // TODO: Make these topics parameters
    m_grid_publisher = create_publisher<nav_msgs::msg::OccupancyGrid>("nav/grid", 10);
    m_pose_publisher = create_publisher<geometry_msgs::msg::Pose>("nav/pose", 10);
    m_pose_array_publisher = create_publisher<geometry_msgs::msg::PoseArray>("nav/particles", 10);
    m_pose_with_covariance_publisher =
        create_publisher<geometry_msgs::msg::PoseWithCovariance>("nav/pose_var", 10);

    m_mcl = std::make_unique<slam::MCL>(n_particles, cv::Size(500, 500));
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
