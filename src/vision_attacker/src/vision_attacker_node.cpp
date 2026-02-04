
#include "vision_attacker/vision_attacker_node.hpp"
#include "vision_attacker/planner.hpp"
#include "vision_attacker/vision_attacker_node.hpp"

// Tracker_node 构造函数实现
Tracker_node::Tracker_node(std::string node_name) : rclcpp::Node(node_name)
{
    lagTime = declare_parameter("trajectory.lag_time", 0.08);
    airK = declare_parameter("trajectory.air_k", 0.04);
    yawFix = declare_parameter("trajectory.yaw_fix", 1.0);
    pitchFix = declare_parameter("trajectory.pitch_fix", 0.0);
    carThreshold = declare_parameter("fire_ctrl.car_attack_threshold", 30.0);
    outpostThreshold = declare_parameter("fire_ctrl.outpost_attack_threshold", 5.0);
    thresholdFix = declare_parameter("fire_ctrl.threshold_fix", 0.0);

    //solver
    double max_yaw_acc = declare_parameter("solver.max_yaw_acc", 100.0);
    double max_pitch_acc = declare_parameter("solver.max_pitch_acc", 100.0);
    std::vector<double> Q_yaw_v = declare_parameter("solver.Q_yaw", std::vector<double>{0.1});
    std::vector<double> R_yaw_v = declare_parameter("solver.R_yaw", std::vector<double>{0.1});
    std::vector<double> Q_pitch_v = declare_parameter("solver.Q_pitch", std::vector<double>{0.1});
    std::vector<double> R_pitch_v = declare_parameter("solver.R_pitch", std::vector<double>{0.1});
    //planner
    double yaw_offset_ = declare_parameter("planner.yaw_offset", 0.0);
    double pitch_offset_ = declare_parameter("planner.pitch_offset", 0.0);
    double fire_thresh_ = declare_parameter("planner.fire_thresh", 0.0);
    double low_speed_delay_time_ = declare_parameter("planner.low_speed_delay_time", 0.0);
    double high_speed_delay_time_ = declare_parameter("planner.high_speed_delay_time", 0.0);
    double decision_speed_ = declare_parameter("planner.decision_speed", 0.0);

    planner_ = std::make_unique<auto_aim::Planner>(
        yaw_offset_, pitch_offset_, fire_thresh_, low_speed_delay_time_, high_speed_delay_time_, decision_speed_,
        max_yaw_acc, max_pitch_acc, Q_yaw_v, R_yaw_v, Q_pitch_v, R_pitch_v,
        lagTime, airK);
    
    robotPtr = std::make_unique<vision_interfaces::msg::Robot>();
    markerPub = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
    aimMarkerPub = this->create_publisher<visualization_msgs::msg::Marker>("/real_aiming_point", 10);
    aimPub = create_publisher<vision_interfaces::msg::AutoAim>(
        "/serial_driver/aim_target", rclcpp::SensorDataQoS());
    robotSub = create_subscription<vision_interfaces::msg::Robot>(
        "/serial_driver/robot", rclcpp::SensorDataQoS(), std::bind(&Tracker_node::robot_callback, this, std::placeholders::_1));
    targetSub = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "/tracker/target", rclcpp::SensorDataQoS(), std::bind(&Tracker_node::target_callback, this, std::placeholders::_1));

    aimPoint.header.frame_id = "odom";
    aimPoint.ns = "aiming_point";
    aimPoint.type = visualization_msgs::msg::Marker::SPHERE;
    aimPoint.action = visualization_msgs::msg::Marker::ADD;
    aimPoint.scale.x = aimPoint.scale.y = aimPoint.scale.z = 0.12;
    aimPoint.color.r = 1.0;
    aimPoint.color.g = 1.0;
    aimPoint.color.b = 1.0;
    aimPoint.color.a = 1.0;
    aimPoint.lifetime = rclcpp::Duration::from_seconds(0.1);

    realAimPoint.header.frame_id = "odom";
    realAimPoint.ns = "real_aiming_point";
    realAimPoint.type = visualization_msgs::msg::Marker::SPHERE;
    realAimPoint.action = visualization_msgs::msg::Marker::ADD;
    realAimPoint.scale.x = realAimPoint.scale.y = realAimPoint.scale.z = 0.12;
    realAimPoint.color.r = 1.0;
    realAimPoint.color.g = 0.0;
    realAimPoint.color.b = 0.0;
    realAimPoint.color.a = 1.0;
    realAimPoint.lifetime = rclcpp::Duration::from_seconds(0.1);
    RCLCPP_INFO(get_logger(), "Vision Attacker Node Initialized.");
}

void Tracker_node::robot_callback(const vision_interfaces::msg::Robot robot)
{
    try
    {
        *robotPtr = robot;
    }
    catch (std::exception &ex)
    {
        RCLCPP_ERROR(get_logger(), "获取机器人信息时发生错误.");
    }
}

void Tracker_node::target_callback(const auto_aim_interfaces::msg::Target target_msg)
{
    lagTime = get_parameter("trajectory.lag_time").as_double();
    airK = get_parameter("trajectory.air_k").as_double();
    yawFix = get_parameter("trajectory.yaw_fix").as_double();
    pitchFix = get_parameter("trajectory.pitch_fix").as_double();
    carThreshold = get_parameter("fire_ctrl.car_attack_threshold").as_double();
    outpostThreshold = get_parameter("fire_ctrl.outpost_attack_threshold").as_double();
    thresholdFix = get_parameter("fire_ctrl.threshold_fix").as_double();
    // Prepare default aim message
    vision_interfaces::msg::AutoAim aim;
    aim.aim_yaw = robotPtr->self_yaw;
    aim.aim_pitch = robotPtr->self_pitch;
    aim.fire = 0;

    if (!target_msg.tracking)
    {
        aim.tracking = 0;
        aimPub->publish(aim);
        RCLCPP_INFO(get_logger(), "aimYaw:%05.2f/aimPitch:%05.2f", aim.aim_yaw, aim.aim_pitch);
        return;
    }

    // Convert tracker message to planner Target and run planner
    auto_aim::Target t;
    t.from_msg(target_msg, std::chrono::steady_clock::now());
    double bullet_speed = robotPtr->muzzle_speed > 15 ? robotPtr->muzzle_speed : 15.0;
    auto plan = planner_->plan(t, bullet_speed);

    // Map planner output (radians) to message (degrees)
    aim.aim_yaw = plan.target_yaw * 180.0 / M_PI;
    // normalize to [0,360)
    if (aim.aim_yaw < 0) aim.aim_yaw += 360.0;
    if (aim.aim_yaw >= 360.0) aim.aim_yaw -= 360.0;
    aim.aim_pitch = plan.target_pitch * 180.0 / M_PI;
    aim.fire = plan.fire ? 1 : 0;
    aim.tracking = 1;
    

    // Update markers from planner debug info
    aimPoint.pose.position.x = planner_->debug_xyza[0];
    aimPoint.pose.position.y = planner_->debug_xyza[1];
    aimPoint.pose.position.z = planner_->debug_xyza[2];

    realAimPoint.pose.position.x = target_msg.position.x - target_msg.radius_1 * cos(robotPtr->self_yaw / 180.0 * M_PI);
    realAimPoint.pose.position.y = target_msg.position.y - target_msg.radius_1 * sin(robotPtr->self_yaw / 180.0 * M_PI);
    realAimPoint.pose.position.z = target_msg.position.z;

    aimPoint.header.stamp = now();
    markerPub->publish(aimPoint);
    aimMarkerPub->publish(realAimPoint);

    aim.aim_yaw += yawFix;
    aim.aim_pitch += pitchFix;
    aim.aim_p_vel = plan.target_pitch_vel * 180.0 / M_PI;
    aim.aim_y_vel = plan.target_yaw_vel * 180.0 / M_PI;
    aimPub->publish(aim);
    
}

// The trajectory solver is implemented as a package-level free function in
// `trajectory.cpp`. Node no longer defines `Tracker_node::solveTrajectory`.

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Tracker_node>("vision_attacker");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
