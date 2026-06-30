#pragma once

#include "simulation/Simulation.hpp"

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include <variant>

class SimBridge : public rclcpp::Node
{

public:
    SimBridge(Sim::Simulation* sim);

protected:
    geometry_msgs::msg::Pose _poseFromRotationAndTranslation(const Mat3r& R, const Vec3r& t) const;

    Sim::Simulation* _sim;

    using LinearRod = SimObject::XPBDRod_<SimObject::RodElement<1>>;
    using QuadraticRod = SimObject::XPBDRod_<SimObject::RodElement<2>>;
    using CubicRod = SimObject::XPBDRod_<SimObject::RodElement<3>>;

    std::variant<LinearRod*, QuadraticRod*, CubicRod*> _rod;
    Constraint::OneSidedFixedJointConstraint* _base_constraint;
    Constraint::OneSidedFixedJointConstraint* _tip_constraint;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _rod_frames_publisher;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _rod_base_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _rod_tip_pose_subscriber;

};