#include "simobject/group/XPBDConcentricTubeRobot.hpp"

#include "config/RodConfig.hpp"

#include "constraint/PointLineConstraint.hpp"

namespace SimObject
{

XPBDConcentricTubeRobot::XPBDConcentricTubeRobot(const Config::XPBDConcentricTubeRobotConfig& config)
    : XPBDObjectGroup_Base(config)
{
    _base_position = config.initialPosition();
    _base_orientation = Math::RotMatFromXYZEulerAngles(config.initialRotation());
}

void XPBDConcentricTubeRobot::setup()
{
    Config::RodConfig outer_rod_config(
        "outer", _base_position, Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), true, false,
        1, 0.1, 10, 1000, 1e6, 0.3
    );

    SimObject::CircleCrossSection outer_cross_section(0.1/2.0, 10);

    Config::RodConfig inner_rod_config(
        "inner", _base_position, Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), true, false,
        1.5, 0.08, 10, 1000, 1e6, 0.3
    );

    SimObject::CircleCrossSection inner_cross_section(0.08/2.0, 10);

    _objects.template reserve<XPBDRod>(2);
    _outer_rod = &_addObject<XPBDRod>(outer_rod_config, outer_cross_section);
    _inner_rod = &_addObject<XPBDRod>(inner_rod_config, inner_cross_section);

    _outer_rod->setup();
    _inner_rod->setup();

    // create mapping between rod nodes and global index and construct inverse inertia matrix
    _M_inv = VecXr::Zero(6*_outer_rod->nodes().size() + 6*_inner_rod->nodes().size());
    int global_index = 0;
    for (const auto& particle : _outer_rod->nodes())
    {
        _M_inv.block<3,1>(global_index*6, 0) = Vec3r::Ones() / particle.mass;
        _M_inv.block<3,1>(global_index*6+3, 0) = Vec3r(1/particle.Ib[0], 1/particle.Ib[1], 1/particle.Ib[2]);
        _particle_ptr_to_index.insert({&particle, global_index++});
    }
    for (const auto& particle : _inner_rod->nodes())
    {
        _M_inv.block<3,1>(global_index*6, 0) = Vec3r::Ones() / particle.mass;
        _M_inv.block<3,1>(global_index*6+3, 0) = Vec3r(1/particle.Ib[0], 1/particle.Ib[1], 1/particle.Ib[2]);
        _particle_ptr_to_index.insert({&particle, global_index++});
    }

    // create initial alpha vector
    _alpha = VecXr::Zero(6*_outer_rod->rodConstraints().size() + 6*_inner_rod->rodConstraints().size());
    int constraint_index = 0;
    for (const auto& constraint : _outer_rod->rodConstraints())
    {
        _alpha.block<6,1>(6*constraint_index++, 0) = constraint.alpha();
    }
    for (const auto& constraint : _inner_rod->rodConstraints())
    {
        _alpha.block<6,1>(6*constraint_index++, 0) = constraint.alpha();
    }



    // create the initial point-on-line constraints
    _updateConcentricityConstraints();
    
}

void XPBDConcentricTubeRobot::_updateConcentricityConstraints()
{
    // for now, clear the point-on-line constraints every time
    _internal_constraints.template clear<Constraint::PointLineConstraint>();

    std::vector<OrientedParticle>& outer_nodes = _outer_rod->nodes();
    std::vector<OrientedParticle>& inner_nodes = _inner_rod->nodes();

    // go through each inner node and project it onto a line segment in the outer tube
    for (auto& inner_node : inner_nodes)
    {
        // project onto the first segment of the outer tube - if t is negative we are outside of the outer tube
        Real t1 = Math::projectPointOntoLine(inner_node.position, outer_nodes[0].position, outer_nodes[1].position);
        if (t1 < 0)
            continue;

        // project onto the last segment of the outer tube - if t is >1 we are outside of the outer tube
        Real t2 = Math::projectPointOntoLine(inner_node.position, outer_nodes[outer_nodes.size()-2].position, outer_nodes.back().position);
        if (t2 > 1)
            continue;

        // this inner node is somewhere inside the outer tube - now we need to find which segment
        Real t = t1;
        int outer_node_index = 0;
        const int MAX_ITERS = 5;
        for (int i = 0; i < MAX_ITERS; i++)
        {
            outer_node_index += static_cast<int>(std::floor(t));    // round towards negative infinity
            outer_node_index = std::clamp(outer_node_index, 0, static_cast<int>(outer_nodes.size())-2);   // and clamp
            
            // std::cout << "Outer node index: " << outer_node_index << std::endl;

            t = Math::projectPointOntoLine(inner_node.position, outer_nodes[outer_node_index].position, outer_nodes[outer_node_index+1].position);

            if (t >= 0 && t <= 1)
                break;
        }

        // create a point-on-line constraint between the inner tube node and the outer tube segment
        _addInternalConstraint<Constraint::PointLineConstraint>(&inner_node, &outer_nodes[outer_node_index], &outer_nodes[outer_node_index+1]);
    }
}

void XPBDConcentricTubeRobot::internalConstraintSolve(Real dt)
{

    // _updateConcentricityConstraints();

    const std::vector<Constraint::RodElasticConstraint>& outer_rod_constraints = _outer_rod->rodConstraints();
    // std::cout << "Num outer rod constraints: " << outer_rod_constraints.size() << std::endl;
    const std::vector<Constraint::RodElasticConstraint>& inner_rod_constraints = _inner_rod->rodConstraints();
    // std::cout << "Num inner rod constraints: " << inner_rod_constraints.size() << std::endl;

    // calculate number of constraints
    int num_constraints = outer_rod_constraints.size()*Constraint::RodElasticConstraint::ConstraintDim +
                          inner_rod_constraints.size()*Constraint::RodElasticConstraint::ConstraintDim + 
                          _internal_constraints.template get<Constraint::PointLineConstraint>().size()*Constraint::PointLineConstraint::ConstraintDim;
    
    _RHS_vec.conservativeResize(num_constraints, 1);
    _alpha.conservativeResize(num_constraints, 1);
    _delC_mat.conservativeResize(num_constraints, _outer_rod->nodes().size()*6 + _inner_rod->nodes().size()*6);
    _LHS_mat.conservativeResize(_outer_rod->nodes().size()*6 + _inner_rod->nodes().size()*6, _outer_rod->nodes().size()*6 + _inner_rod->nodes().size()*6 );
    // start assembling global system
    int row_offset = 0;
    for (const auto& constraint : outer_rod_constraints)
    {
        Vec6r C = constraint.evaluate();
        _RHS_vec.block<6,1>(row_offset, 0) = -C;

        Constraint::RodElasticConstraint::GradientMatType grad = constraint.gradient();
        int pos1_index = _particle_ptr_to_index.at(constraint.particles()[0]);
        int pos2_index = _particle_ptr_to_index.at(constraint.particles()[1]);
        _delC_mat.block<6,6>(row_offset, pos1_index*6) = grad.block<6,6>(0,0);
        _delC_mat.block<6,6>(row_offset, pos2_index*6) = grad.block<6,6>(0,6);

        row_offset += Constraint::RodElasticConstraint::ConstraintDim;
    }

    for (const auto& constraint : inner_rod_constraints)
    {
        Vec6r C = constraint.evaluate();
        _RHS_vec.block<6,1>(row_offset, 0) = -C;

        Constraint::RodElasticConstraint::GradientMatType grad = constraint.gradient();
        int pos1_index = _particle_ptr_to_index.at(constraint.particles()[0]);
        int pos2_index = _particle_ptr_to_index.at(constraint.particles()[1]);
        _delC_mat.block<6,6>(row_offset, pos1_index*6) = grad.block<6,6>(0,0);
        _delC_mat.block<6,6>(row_offset, pos2_index*6) = grad.block<6,6>(0,6);

        row_offset += Constraint::RodElasticConstraint::ConstraintDim;
    }

    for (const auto& constraint : _internal_constraints.template get<Constraint::PointLineConstraint>())
    {
        Real C = constraint.evaluate()[0];
        _RHS_vec[row_offset] = -C;

        Constraint::PointLineConstraint::GradientMatType grad = constraint.gradient();
        int pos1_index = _particle_ptr_to_index.at(constraint.particles()[0]);
        int pos2_index = _particle_ptr_to_index.at(constraint.particles()[1]);
        int pos3_index = _particle_ptr_to_index.at(constraint.particles()[2]);
        _delC_mat.block<1,6>(row_offset, pos1_index*6) = grad.block<1,6>(0,0);
        _delC_mat.block<1,6>(row_offset, pos2_index*6) = grad.block<1,6>(0,6);
        _delC_mat.block<1,6>(row_offset, pos3_index*6) = grad.block<1,6>(0,12);

        _alpha[row_offset] = constraint.alpha()[0];

        row_offset += Constraint::PointLineConstraint::ConstraintDim;
    }


    // compute LHS
    // std::cout << "_delC_mat: \n" << _delC_mat << std::endl;
    // std::cout << "_delC size: " << _delC_mat.rows() << " x " << _delC_mat.cols() << "  _M_inv size: " << _M_inv.size() << std::endl;
    _LHS_mat = _delC_mat * _M_inv.asDiagonal() * _delC_mat.transpose();
    // std::cout << "_LHS_mat size: " << _LHS_mat.rows() << " x " << _LHS_mat.cols() << std::endl;
    // std::cout << "_alpha size: " << _alpha.size() << std::endl;
    _LHS_mat.diagonal() += _alpha/(dt*dt);

    // solve and compute position update
    _dlam = _LHS_mat.llt().solve(_RHS_vec);
    // std::cout << "_dlam size: " << _dlam.size() << std::endl;
    _dx = _M_inv.asDiagonal() * _delC_mat.transpose() * _dlam;
    // std::cout << "Position update: " << _dx.transpose() << std::endl;
    // std::cout << "_dx size: " << _dx.size() << std::endl;

    // update positions
    for (int i = 0; i < _outer_rod->nodes().size(); i++)
    {
        const Vec3r dp = _dx( Eigen::seqN(6*i,3) );
        const Vec3r dor = _dx( Eigen::seqN(6*i+3,3) );
        _outer_rod->nodes()[i].positionUpdate(dp, dor);
    }

    for (int i = 0; i < _inner_rod->nodes().size(); i++)
    {
        const Vec3r dp = _dx( Eigen::seqN(6*(i+_outer_rod->nodes().size()),3) );
        const Vec3r dor = _dx( Eigen::seqN(6*(i+_outer_rod->nodes().size())+3,3) );
        _inner_rod->nodes()[i].positionUpdate(dp, dor);
    }
}

} // namespace SimObject