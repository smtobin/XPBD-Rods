#pragma once

#include "simulation/SuturingSimulation.hpp"

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include <variant>

class SuturingSimBridge : public rclcpp::Node
{

public:
    using LinearRod = SimObject::XPBDRod_<SimObject::RodElement<1>>;
    using QuadraticRod = SimObject::XPBDRod_<SimObject::RodElement<2>>;
    using CubicRod = SimObject::XPBDRod_<SimObject::RodElement<3>>;
    using RodVariant = std::variant<LinearRod*, QuadraticRod*, CubicRod*>;

    SuturingSimBridge(Sim::SuturingSimulation* sim);

protected:
    

    geometry_msgs::msg::Pose _poseFromRotationAndTranslation(const Mat3r& R, const Vec3r& t) const;
    void _createGraspPoseSubscribers();
    void _createGraspStateSubscribers();
    void _createRodFramesPublisher(RodVariant& rod_var);

    Sim::SuturingSimulation* _sim;

    std::vector<RodVariant> _rods;

    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr> _thread_frames_publishers;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _straight_tool_grasp_pose_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _curved_tool_grasp_pose_subscriber;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _straight_tool_grasp_subscriber;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _curved_tool_grasp_subscriber;

};