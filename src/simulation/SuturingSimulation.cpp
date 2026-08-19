#include "simulation/SuturingSimulation.hpp"

namespace Sim
{

SuturingSimulation::SuturingSimulation()
    : Simulation()
{}

SuturingSimulation::SuturingSimulation(const Config::SimulationConfig& config)
    : Simulation(config)
{
    _straight_tool_tip_offset = Vec3r(130,0,0);
    _curved_tool_tip_offset = Vec3r(130,0,0);
}

void SuturingSimulation::notifyKeyPressed(const std::string& key)
{
    Simulation::notifyKeyPressed(key);

    if (key == "Shift_L")
    {
        addCallback([this]() { this->_toggleStraightToolGrasping(); });
    }
    if (key == "Shift_R")
    {
        addCallback([this]() { this->_toggleCurvedToolGrasping(); });
    }

    // std::cout << "Key pressed: " << key << std::endl;
}

void SuturingSimulation::notifyKeyReleased(const std::string& key)
{
    Simulation::notifyKeyReleased(key);

    // std::cout << "Key released: " << key << std::endl;
}

void SuturingSimulation::setup()
{
    Simulation::setup();

    // reserve space
    _objects.template reserve<std::unique_ptr<LinearRod>>(2);
    _objects.template reserve<std::unique_ptr<QuadraticRod>>(2);
    _objects.template reserve<std::unique_ptr<SimObject::XPBDRigidMesh>>(2);

    // create threads
    Config::RodConfig thread1_config(
        "thread1",
        Vec3r(0, 1, 0.15),
        Vec3r(0, 0, 0),
        Vec3r(0,0,0),
        Vec3r(0,0,0),
        true,
        0.5,
        0.2,
        Config::RodElementType::LINEAR,
        true,
        false,
        true,
        125,    // length
        0.2,    // diameter
        200,    // num elements
        1.1e-6, // density
        2000,   // E
        0.4,    // nu
        1e5,    // beta
        Vec3r(0,0,0)    // curvature
    );
    thread1_config.renderConfig().setColor(Vec3r(1.0, 0.0, 0.0));
    _addObjectFromConfig(thread1_config);
    _thread1 = _objects.template get<std::unique_ptr<LinearRod>>().back().get();

    Config::RodConfig thread2_config(
        "thread2",
        Vec3r(0, 1, -0.15),
        Vec3r(0, 180, 0),
        Vec3r(0,0,0),
        Vec3r(0,0,0),
        true,
        0.5,
        0.2,
        Config::RodElementType::LINEAR,
        true,
        false,
        true,
        65,    // length
        0.2,    // diameter
        100,    // num elements
        1.1e-6, // density
        2000,   // E
        0.4,    // nu
        1e5,    // beta
        Vec3r(0,0,0)    // curvature
    );
    thread2_config.renderConfig().setColor(Vec3r(0.0, 1.0, 0.0));
    _addObjectFromConfig(thread2_config);
    _thread2 = _objects.template get<std::unique_ptr<LinearRod>>().back().get();

    // update the fixed base constraints on thread1 and thread2 so that they go into the ground
    auto& base_fixed_constraint1 = _thread1->internalConstraints().template get<Constraint::OneSidedFixedJointConstraint>().back();
    base_fixed_constraint1.setReferencePosition(Vec3r(0, 0.01, 1.0));
    base_fixed_constraint1.setReferenceOrientation(Math::RotMatFromXYZEulerAngles(Vec3r(-90,0,0)));
    auto& base_fixed_constraint2 = _thread2->internalConstraints().template get<Constraint::OneSidedFixedJointConstraint>().back();
    base_fixed_constraint2.setReferencePosition(Vec3r(0, 0.01, -1.0));
    base_fixed_constraint2.setReferenceOrientation(Math::RotMatFromXYZEulerAngles(Vec3r(-90,0,0)));


    // create tools
    Config::XPBDRigidMeshConfig straight_tool_config(
        "straight_tool",
        Vec3r(0, 150, 50),
        Vec3r(0,0,180),
        Vec3r(0,0,0),
        Vec3r(0,0,0),
        true,
        0.5,
        0.2,
        1000,
        true,
        "../resource/meshes/Scissor_Straight.stl",
        Vec3r(1,1,1)
    );
    _addObjectFromConfig(straight_tool_config);
    _straight_tool = _objects.template get<std::unique_ptr<SimObject::XPBDRigidMesh>>().back().get();

    Config::XPBDRigidSphereConfig straight_tool_grasp_sphere_config(
        "straight_tool_grasp_sphere",
        _straight_tool->com().position + _straight_tool->com().orientation * _straight_tool_tip_offset,
        Vec3r(0,0,0),
        Vec3r(0,0,0),
        Vec3r(0,0,0),
        false,
        0.0,
        0.0,
        1000,
        true,
        10  
    );
    straight_tool_grasp_sphere_config.renderConfig().setColor(Vec3r(1.0, 1.0, 0.0));
    straight_tool_grasp_sphere_config.renderConfig().setOpacity(0.3);
    _addObjectFromConfig(straight_tool_grasp_sphere_config);
    _straight_tool_grasp_sphere = _objects.template get<std::unique_ptr<SimObject::XPBDRigidSphere>>().back().get();

    Config::XPBDRigidMeshConfig curved_tool_config(
        "curved_tool",
        Vec3r(0, 200, 50),
        Vec3r(0,0,180),
        Vec3r(0,0,0),
        Vec3r(0,0,0),
        true,
        0.5,
        0.2,
        1000,
        true,
        "../resource/meshes/Scissor_Curved.stl",
        Vec3r(1,1,1)
    );
    _addObjectFromConfig(curved_tool_config);
    _curved_tool = _objects.template get<std::unique_ptr<SimObject::XPBDRigidMesh>>().back().get();

    Config::XPBDRigidSphereConfig curved_tool_grasp_sphere_config(
        "curved_tool_grasp_sphere",
        _curved_tool->com().position + _curved_tool->com().orientation * _curved_tool_tip_offset,
        Vec3r(0,0,0),
        Vec3r(0,0,0),
        Vec3r(0,0,0),
        false,
        0.0,
        0.0,
        1000,
        true,
        10  
    );
    curved_tool_grasp_sphere_config.renderConfig().setColor(Vec3r(1.0, 1.0, 0.0));
    curved_tool_grasp_sphere_config.renderConfig().setOpacity(0.3);
    _addObjectFromConfig(curved_tool_grasp_sphere_config);
    _curved_tool_grasp_sphere = _objects.template get<std::unique_ptr<SimObject::XPBDRigidSphere>>().back().get();

}

void SuturingSimulation::_timeStep()
{
    _updateToolPositionsFromKeyboard();

    Simulation::_timeStep();

    // if (!_straight_tool_grasping)
    // {
    //     _thread1->addFixedMidConstraint(50, 0.5, Vec3r(0,10,10), Mat3r::Identity());
    //     _straight_tool_grasping = true;
    // }
}

void SuturingSimulation::_updateStraightToolPose(const Vec3r& new_pos, const Mat3r& new_rot)
{
    _straight_tool->com().position = new_pos;
    _straight_tool->com().orientation = new_rot;
    Vec3r straight_tool_tip_pos = new_pos + new_rot * _straight_tool_tip_offset;
    if (_straight_tool_grasping)
    {
        _straight_tool_rod_constraint->setReferenceOrientation(new_rot * _straight_tool_grasp_rot_offset);
        _straight_tool_rod_constraint->setReferencePosition(straight_tool_tip_pos);
    }
    _straight_tool_grasp_sphere->com().position = straight_tool_tip_pos;
}

void SuturingSimulation::_updateCurvedToolPose(const Vec3r& new_pos, const Mat3r& new_rot)
{
    _curved_tool->com().position = new_pos;
    _curved_tool->com().orientation = new_rot;
    Vec3r curved_tool_tip_pos = new_pos + new_rot * _curved_tool_tip_offset;
    if (_curved_tool_grasping)
    {
        _curved_tool_rod_constraint->setReferenceOrientation(new_rot * _curved_tool_grasp_rot_offset);
        _curved_tool_rod_constraint->setReferencePosition(curved_tool_tip_pos);
    }
    _curved_tool_grasp_sphere->com().position = curved_tool_tip_pos;
}

void SuturingSimulation::_updateToolPositionsFromKeyboard()
{
    // copy keys held to avoid race condition with event handling
    std::unordered_set<std::string> keys_held;
    {
        std::lock_guard<std::mutex> l(_keys_mutex);
        keys_held = _keys_held;
    }

    if (keys_held.empty())
        return;

    Real position_delta = 100*_dt;
    Real rotation_delta = 2*_dt;

    Vec3r straight_dp = Vec3r::Zero();
    if (keys_held.count("q") > 0)
        straight_dp += Vec3r(position_delta, 0, 0);
    if (keys_held.count("w") > 0)
        straight_dp -= Vec3r(position_delta, 0, 0);
    if (keys_held.count("a") > 0)
        straight_dp += Vec3r(0, position_delta, 0);
    if (keys_held.count("s") > 0)
        straight_dp -= Vec3r(0, position_delta, 0);
    if (keys_held.count("z") > 0)
        straight_dp += Vec3r(0, 0, position_delta);
    if (keys_held.count("x") > 0)
        straight_dp -= Vec3r(0, 0, position_delta);

    Vec3r straight_dR = Vec3r::Zero();
    if (keys_held.count("e") > 0)
        straight_dR += Vec3r(rotation_delta, 0, 0);
    if (keys_held.count("r") > 0)
        straight_dR -= Vec3r(rotation_delta, 0, 0);
    if (keys_held.count("d") > 0)
        straight_dR += Vec3r(0, rotation_delta, 0);
    if (keys_held.count("f") > 0)
        straight_dR -= Vec3r(0, rotation_delta, 0);
    if (keys_held.count("c") > 0)
        straight_dR += Vec3r(0, 0, rotation_delta);
    if (keys_held.count("v") > 0)
        straight_dR -= Vec3r(0, 0, rotation_delta);

    Mat3r straight_new_R = _straight_tool->com().orientation * Math::Exp_so3(straight_dR);
    Vec3r straight_tip_pos = _straight_tool->com().position + _straight_tool->com().orientation * _straight_tool_tip_offset;
    _updateStraightToolPose(straight_tip_pos - straight_new_R * _straight_tool_tip_offset + straight_dp, straight_new_R);


    Vec3r curved_dp = Vec3r::Zero();
    if (keys_held.count("t") > 0)
        curved_dp += Vec3r(position_delta, 0, 0);
    if (keys_held.count("y") > 0)
        curved_dp -= Vec3r(position_delta, 0, 0);
    if (keys_held.count("g") > 0)
        curved_dp += Vec3r(0, position_delta, 0);
    if (keys_held.count("h") > 0)
        curved_dp -= Vec3r(0, position_delta, 0);
    if (keys_held.count("b") > 0)
        curved_dp += Vec3r(0, 0, position_delta);
    if (keys_held.count("n") > 0)
        curved_dp -= Vec3r(0, 0, position_delta);

    Vec3r curved_dR = Vec3r::Zero();
    if (keys_held.count("u") > 0)
        curved_dR += Vec3r(rotation_delta, 0, 0);
    if (keys_held.count("i") > 0)
        curved_dR -= Vec3r(rotation_delta, 0, 0);
    if (keys_held.count("j") > 0)
        curved_dR += Vec3r(0, rotation_delta, 0);
    if (keys_held.count("k") > 0)
        curved_dR -= Vec3r(0, rotation_delta, 0);
    if (keys_held.count("m") > 0)
        curved_dR += Vec3r(0, 0, rotation_delta);
    if (keys_held.count("comma") > 0)
        curved_dR -= Vec3r(0, 0, rotation_delta);

    Mat3r curved_new_R = _curved_tool->com().orientation * Math::Exp_so3(curved_dR);
    Vec3r curved_tip_pos = _curved_tool->com().position + _curved_tool->com().orientation * _curved_tool_tip_offset;
    _updateCurvedToolPose(curved_tip_pos - curved_new_R * _curved_tool_tip_offset + curved_dp, curved_new_R);
}

void SuturingSimulation::_toggleStraightToolGrasping()
{
    if (!_straight_tool_grasping)
    {
        // search both threads and find the closest point on
        int elem1, elem2;
        Real s_hat1, s_hat2, dist1, dist2;
        _findClosestPointOnRod(_thread1, _straight_tool_grasp_sphere->com().position, elem1, s_hat1, dist1);
        _findClosestPointOnRod(_thread2, _straight_tool_grasp_sphere->com().position, elem2, s_hat2, dist2);
        
        Vec3r tip_pos = _straight_tool->com().position + _straight_tool->com().orientation * _straight_tool_tip_offset;
        if (dist1 < _straight_tool_grasp_sphere->radius())
        {
            Mat3r rod_R = _thread1->elements()[elem1].orientation(s_hat1);
            _straight_tool_grasp_rot_offset = _straight_tool->com().orientation.transpose() * rod_R;

            _thread1->addFixedMidConstraint(elem1, s_hat1, tip_pos, rod_R);
            _straight_tool_rod_constraint = &_thread1->internalConstraints().template get<Constraint::RodMidElementFixedConstraint<SimObject::RodElement<1>>>().back();
            _straight_tool_grasping = true;
            _straight_tool_grasped_rod = _thread1;
        }
        else if (dist2 < _straight_tool_grasp_sphere->radius())
        {
            Mat3r rod_R = _thread2->elements()[elem2].orientation(s_hat2);
            _straight_tool_grasp_rot_offset = _straight_tool->com().orientation.transpose() * rod_R;

            _thread2->addFixedMidConstraint(elem2, s_hat2, tip_pos, _thread2->elements()[elem2].orientation(s_hat2));
            _straight_tool_rod_constraint = &_thread2->internalConstraints().template get<Constraint::RodMidElementFixedConstraint<SimObject::RodElement<1>>>().back();
            _straight_tool_grasping = true;
            _straight_tool_grasped_rod = _thread2;
        }
    }
    else
    {
        _straight_tool_grasped_rod->removeFixedMidConstraint();
        _straight_tool_grasping = false;
    }
}

void SuturingSimulation::_toggleCurvedToolGrasping()
{
    if (!_curved_tool_grasping)
    {
        // search both threads and find the closest point on
        int elem1, elem2;
        Real s_hat1, s_hat2, dist1, dist2;
        Vec3r tip_pos = _curved_tool->com().position + _curved_tool->com().orientation * _curved_tool_tip_offset;

        _findClosestPointOnRod(_thread1, tip_pos, elem1, s_hat1, dist1);
        _findClosestPointOnRod(_thread2, tip_pos, elem2, s_hat2, dist2);
        
        
        if (dist1 < _curved_tool_grasp_sphere->radius())
        {
            Mat3r rod_R = _thread1->elements()[elem1].orientation(s_hat1);
            _curved_tool_grasp_rot_offset = _curved_tool->com().orientation.transpose() * rod_R;

            _thread1->addFixedMidConstraint(elem1, s_hat1, tip_pos, rod_R);
            _curved_tool_rod_constraint = &_thread1->internalConstraints().template get<Constraint::RodMidElementFixedConstraint<SimObject::RodElement<1>>>().back();
            _curved_tool_grasping = true;
            _curved_tool_grasped_rod = _thread1;
        }
        else if (dist2 < _curved_tool_grasp_sphere->radius())
        {
            Mat3r rod_R = _thread2->elements()[elem2].orientation(s_hat2);
            _curved_tool_grasp_rot_offset = _curved_tool->com().orientation.transpose() * rod_R;

            _thread2->addFixedMidConstraint(elem2, s_hat2, tip_pos, _thread2->elements()[elem2].orientation(s_hat2));
            _curved_tool_rod_constraint = &_thread2->internalConstraints().template get<Constraint::RodMidElementFixedConstraint<SimObject::RodElement<1>>>().back();
            _curved_tool_grasping = true;
            _curved_tool_grasped_rod = _thread2;
        }
    }
    else
    {
        _curved_tool_grasped_rod->removeFixedMidConstraint();
        _curved_tool_grasping = false;
    }
}

void SuturingSimulation::_findClosestPointOnRod(LinearRod* rod, const Vec3r& p, int& element_ind, Real& s_hat, Real& dist)
{
    const auto& elements = rod->elements();

    // coarse search through all elements to find the element with the closest node
    dist = std::numeric_limits<Real>::max();
    for (unsigned e = 0; e < elements.size(); e++)
    {
        const auto& element = elements[e];
        for (int i = 0; i < element.numNodes(); i++)
        {
            Real d = (p - element.node(i)->position).norm();
            if (d < dist)
            {
                dist = d;
                element_ind = static_cast<int>(e);
                s_hat = Real(i) / (element.numNodes() - 1);
            }
        }
    }
}

} // namespace Sim