#include "simulation/RodGraspingSimulation.hpp"

namespace Sim
{

RodGraspingSimulation::RodGraspingSimulation()
    : Simulation()
{

}

RodGraspingSimulation::RodGraspingSimulation(const Config::SimulationConfig& sim_config)
    : Simulation(sim_config)
{

}

void RodGraspingSimulation::notifyMouseMoved(double mx, double my)
{
    if (_grasping)
    {
        Vec3r ref_position = _grasping_constraint->referencePosition();
        ref_position[0] += (mx - _last_mx)/100.0;
        ref_position[1] += (my - _last_my)/100.0;

        auto set_reference_position_callback =
            [this, ref_position]() -> void {
                this->_grasping_constraint->setReferencePosition(ref_position);
            };
        _addCallback(set_reference_position_callback);
    }

    _last_mx = mx;
    _last_my = my;
}

void RodGraspingSimulation::notifyLeftMouseButtonPressed()
{
    // toggle grasping
    if (_grasping)
    {
        _grasping = false;

        // remove attachment constraint
        auto remove_attachment_constraint_callback = 
            [this]() -> void {
                this->_grasped_rod->removeAttachmentConstraint(this->_grasped_rod_node->index, this->_grasping_constraint);

                this->_grasped_rod = nullptr;
                this->_grasped_rod_node = nullptr;
                this->_grasping_constraint = nullptr;
            };
        _addCallback(remove_attachment_constraint_callback);
        
    }
    else
    {
        _grasping = true;
        _graspClosestNode();
    }
}

void RodGraspingSimulation::_graspClosestNode()
{
    // convert all rod points to 2D camera coordinates
    Rod::XPBDRod* closest_rod = nullptr;
    const Rod::XPBDRodNode* closest_node = nullptr;
    Real min_dist = std::numeric_limits<Real>::max();
    for (auto& rod : _rods)
    {
        for (const auto& node : rod.nodes())
        {
            Vec2r pix_coords = _graphics_scene.worldCoordinatesToPixelCoordinates(node.position);
            Real dist = (Vec2r(_last_mx, _last_my) - pix_coords).norm();
            if (dist < min_dist)
            {
                closest_rod = &rod;
                closest_node = &node;
                min_dist = dist;
            }
        }
    }

    if (closest_rod && closest_node)
    {
        _grasped_rod = closest_rod;
        _grasped_rod_node = closest_node;
    }

    // add attachment constraint
    auto add_attachment_constraint_callback = 
        [this]() -> void {
            this->_grasping_constraint = this->_grasped_rod->addAttachmentConstraint(
                this->_grasped_rod_node->index, this->_grasped_rod_node->position, this->_grasped_rod_node->orientation
            );
        };
    _addCallback(add_attachment_constraint_callback);
    
        
}

} // namespace Sim