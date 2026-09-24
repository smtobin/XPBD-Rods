#include "sim_bridge/SuturingSimBridge.hpp"

#include "simobject/rod/XPBDHigherOrderRod.hpp"

SuturingSimBridge::SuturingSimBridge(Sim::SuturingSimulation* sim)
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

    _createGraspPoseSubscribers();
    _createGraspStateSubscribers();
    
}

geometry_msgs::msg::Pose SuturingSimBridge::_poseFromRotationAndTranslation(const Mat3r& R, const Vec3r& t) const
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

void SuturingSimBridge::_createRodFramesPublisher(RodVariant& rod_var)
{
    std::string rod_name = std::visit([&](const auto& rod) { return rod->name(); }, rod_var);
    std::string topic_name = "sim/" + rod_name + "/frames";
    auto new_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(topic_name, 10);
    _thread_frames_publishers.push_back(new_pub);
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

void SuturingSimBridge::_createGraspPoseSubscribers()
{
    std::string straight_tool_pose_topic = "/sim/straight_tool/grasp_pose";
    std::string curved_tool_pose_topic = "/sim/curved_tool/grasp_pose";

    auto straight_tool_grasp_pose_callback = [this](geometry_msgs::msg::PoseStamped::UniquePtr msg) -> void 
    {
        Vec3r p = Vec3r(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        Vec4r quat = Vec4r(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        Mat3r R = Math::quaternionToRotationMatrix(quat);

        auto callback = [this, p, R]() -> void
        {
            this->_sim->updateStraightToolGraspPose(p, R);
        };
        this->_sim->addCallback(callback);
    };

    auto curved_tool_grasp_pose_callback = [this](geometry_msgs::msg::PoseStamped::UniquePtr msg) -> void
    {
        Vec3r p = Vec3r(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        Vec4r quat = Vec4r(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        Mat3r R = Math::quaternionToRotationMatrix(quat);

        auto callback = [this, p, R]() -> void
        {
            this->_sim->updateCurvedToolGraspPose(p, R);
        };
        this->_sim->addCallback(callback);
    };

    _straight_tool_grasp_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(straight_tool_pose_topic, 10, straight_tool_grasp_pose_callback);
    _curved_tool_grasp_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(curved_tool_pose_topic, 10, curved_tool_grasp_pose_callback);
}

void SuturingSimBridge::_createGraspStateSubscribers()
{
    std::string straight_tool_grasp_state_topic = "/sim/straight_tool/grasping";
    std::string curved_tool_grasp_state_topic = "/sim/curved_tool/grasping";

    auto straight_tool_grasp_state_callback = [this](std_msgs::msg::Bool::UniquePtr msg) -> void 
    {
        bool state = msg->data;
        auto callback = [this, state]() -> void
        {
            this->_sim->setStraightToolGrasping(state);
        };
        this->_sim->addCallback(callback);
    };

    auto curved_tool_grasp_state_callback = [this](std_msgs::msg::Bool::UniquePtr msg) -> void
    {
        bool state = msg->data;
        auto callback = [this, state]() -> void
        {
            this->_sim->setCurvedToolGrasping(state);
        };
        this->_sim->addCallback(callback);
    };

    _straight_tool_grasp_subscriber = this->create_subscription<std_msgs::msg::Bool>(straight_tool_grasp_state_topic, 10, straight_tool_grasp_state_callback);
    _curved_tool_grasp_subscriber = this->create_subscription<std_msgs::msg::Bool>(curved_tool_grasp_state_topic, 10, curved_tool_grasp_state_callback);
}