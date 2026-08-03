#include "sim_bridge/SimBridge.hpp"

#include "simobject/rod/XPBDHigherOrderRod.hpp"

SimBridge::SimBridge(Sim::Simulation* sim)
    : rclcpp::Node("sim_bridge"), _sim(sim)
{
    this->declare_parameter("publish_rate_hz", 30.0);
    this->declare_parameter("num_rod_frames", 10);

    // find the first rod in the sim
    // this is the rod we will publish the state for
    bool rod_found = false;
    const auto& sim_objs = _sim->objects();
    // first look through linear rods
    for (auto& rod : sim_objs.get<std::unique_ptr<LinearRod>>())
    {
        RodVariant rod_var = rod.get();
        _rods.push_back(rod_var);
        rod_found = true;
    }
    // then look through quadratic rods
    for (auto& rod : sim_objs.get<std::unique_ptr<QuadraticRod>>())
    {
        RodVariant rod_var = rod.get();
        _rods.push_back(rod_var);
        rod_found = true;
    }
    // then look through cubic rods
    for (auto& rod : sim_objs.get<std::unique_ptr<CubicRod>>())
    {
        RodVariant rod_var = rod.get();
        _rods.push_back(rod_var);
        rod_found = true;
    }

    if (!rod_found)
        throw std::runtime_error("No rod found in the simulation! Check the config file?");

    // set up rod publishers
    for (auto& rod_var : _rods)
    {
        _createRodFramesPublisher(rod_var);
    }

    // set up rod fixed constraint subscribers
    for (unsigned rod_idx = 0; rod_idx < _rods.size(); rod_idx++)
    {
        _createBaseAndTipConstraintSubscribers(_rods[rod_idx], rod_idx);
    }
    
}

geometry_msgs::msg::Pose SimBridge::_poseFromRotationAndTranslation(const Mat3r& R, const Vec3r& t) const
{
    geometry_msgs::msg::Pose pose;

    pose.position.x = t[0];
    pose.position.y = t[1];
    pose.position.z = t[2];

    Vec4r quat = Math::rotationMatrixToQuaternion(R);
    pose.orientation.x = quat[0];
    pose.orientation.y = quat[1];
    pose.orientation.z = quat[2];
    pose.orientation.w = quat[3];

    return pose;
}

void SimBridge::_createRodFramesPublisher(RodVariant& rod_var)
{
    std::string rod_name = std::visit([&](const auto& rod) { return rod->name(); }, rod_var);
    std::string topic_name = "sim/" + rod_name + "/frames";
    auto new_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(topic_name, 10);
    _rod_frames_publishers.push_back(new_pub);
    auto rod_publish_callback = [this, rod_var, new_pub] () -> void
    {
        int num_frames = this->get_parameter("num_rod_frames").as_int();

        auto message = geometry_msgs::msg::PoseArray();
        message.header.stamp = this->now();
        message.header.frame_id = "ves/left/base";

        std::visit([&](const auto& rod) {
            const auto& elements = rod->elements();

            // sample the rod
            for (int si = 0; si < num_frames; si++)
            {   
                // position along rod in [0, 1]
                Real s = (Real)si / (num_frames - 1);
                // the element index that s corresponds to
                int elem_ind = std::clamp(static_cast<int>(s * elements.size()), 0, static_cast<int>(elements.size()-1));
                // the "local" s within the element
                Real s_hat = s * elements.size() - (Real)elem_ind;

                Vec3r p = elements[elem_ind].position(s_hat);
                Mat3r R = elements[elem_ind].orientation(s_hat);

                message.poses.push_back(this->_poseFromRotationAndTranslation(R, p));
            }
        }, rod_var);

        new_pub->publish(message);
    };

    _sim->addRepeatedCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), rod_publish_callback);
}

void SimBridge::_createBaseAndTipConstraintSubscribers(RodVariant& rod_var, unsigned rod_idx)
{
    std::string rod_name = std::visit([&](const auto& rod) { return rod->name(); }, rod_var);
    std::string base_topic_name = "sim/" + rod_name + "/base_pose";
    std::string tip_topic_name = "sim/" + rod_name + "/tip_pose";

    // create callback for publishing the base and tip pose of the rod
    std::visit([&](const auto& rod) {
        // get internal constraints for the rod
        auto& rod_internal_constraints = rod->internalConstraints();
        auto& fixed_joint_constraints = rod_internal_constraints.template get<Constraint::OneSidedFixedJointConstraint>();

        // make sure there are two fixed constraints (one for the base and one for the tip)
        if (fixed_joint_constraints.size() != 2)
            throw std::runtime_error("Base and/or tip of rod are not fixed! Check the config file?");

        // get the fixed base and tip constraints for the rod
        _base_constraints.push_back(&fixed_joint_constraints.front());
        _tip_constraints.push_back(&fixed_joint_constraints.back());

        auto rod_base_pose_callback = [this, rod_idx](geometry_msgs::msg::PoseStamped::UniquePtr msg) -> void 
        {
            Vec3r p = Vec3r(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            Vec4r quat = Vec4r(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
            Mat3r R = Math::quaternionToRotationMatrix(quat);

            auto callback = [this, rod_idx, p, R]() -> void
            {
                this->_base_constraints[rod_idx]->setReferencePosition(p);
                this->_base_constraints[rod_idx]->setReferenceOrientation(R);
            };
            this->_sim->addCallback(callback);
        };

        auto rod_tip_pose_callback = [this, rod_idx](geometry_msgs::msg::PoseStamped::UniquePtr msg) -> void
        {
            Vec3r p = Vec3r(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
            Vec4r quat = Vec4r(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
            Mat3r R = Math::quaternionToRotationMatrix(quat);

            auto callback = [this, rod_idx, p, R]() -> void
            {
                this->_tip_constraints[rod_idx]->setReferencePosition(p);
                this->_tip_constraints[rod_idx]->setReferenceOrientation(R);
            };
            this->_sim->addCallback(callback);
        };

        auto new_base_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(base_topic_name, 10, rod_base_pose_callback);
        auto new_tip_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(tip_topic_name, 10, rod_tip_pose_callback);
        this->_rod_base_pose_subscribers.push_back(new_base_sub);
        this->_rod_tip_pose_subscribers.push_back(new_tip_sub);
    }, rod_var);
}