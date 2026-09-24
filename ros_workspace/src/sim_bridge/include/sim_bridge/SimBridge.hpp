#pragma once

#include "simulation/Simulation.hpp"

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include <variant>

class SimBridge : public rclcpp::Node
{

public:
    using LinearRod = SimObject::XPBDRod_<SimObject::RodElement<1>>;
    using QuadraticRod = SimObject::XPBDRod_<SimObject::RodElement<2>>;
    using CubicRod = SimObject::XPBDRod_<SimObject::RodElement<3>>;
    using RodVariant = std::variant<LinearRod*, QuadraticRod*, CubicRod*>;

    SimBridge(Sim::Simulation* sim);

protected:
    

    geometry_msgs::msg::Pose _poseFromRotationAndTranslation(const Mat3r& R, const Vec3r& t) const;
    void _createBaseAndTipConstraintSubscribers(RodVariant& rod_var, unsigned rod_idx);
    void _createRodFramesPublisher(RodVariant& rod_var);

    Sim::Simulation* _sim;

    

    std::vector<RodVariant> _rods;
    std::vector<Constraint::OneSidedFixedJointConstraint*> _base_constraints;
    std::vector<Constraint::OneSidedFixedJointConstraint*> _tip_constraints;

    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr> _rod_frames_publishers;

    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> _rod_base_pose_subscribers;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> _rod_tip_pose_subscribers;

};