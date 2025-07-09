#include "rod/XPBDRod.hpp"
#include "common/math.hpp"

#include "constraint/RodElasticConstraint.hpp"

#include <iostream>
#include <variant>

namespace Rod
{

void XPBDRod::setup()
{
    _dl = _length / (_num_nodes - 1);

    Vec6r alpha_elastic = 1/_dl * Vec6r( 1.0 / (_G * _cross_section->area()), 1.0 / (_G * _cross_section->area()), 1.0 / (_E * _cross_section->area()),
                                         1.0 / (_E * _cross_section->Ix()),   1.0 / (_E * _cross_section->Iy()),   1.0 / (_G * _cross_section->Iz()) );
    // create elastic constraints
    _elastic_constraints.reserve(_num_nodes-1);
    for (int i = 0; i < _num_nodes-1; i++)
    {
        _elastic_constraints.emplace_back(&_nodes[i], &_nodes[i+1], alpha_elastic);
    }

    // add a rigid attachment constraint at the base
    _attachment_constraints.emplace(&_nodes[0], Vec6r::Zero(), _nodes[0].position, _nodes[0].orientation);
    // _attachment_constraints.emplace(&_nodes.back(), Vec6r::Zero(), _nodes.back().position, _nodes.back().orientation);
    _attachment_constraints.emplace(&_nodes[_num_nodes/2], Vec6r::Zero(), _nodes[_num_nodes/2].position, _nodes[_num_nodes/2].orientation);

    

    // order constraints appropriately
    int num_diagonals = _orderConstraints();
    // resize diagonals
    _diagonals.resize(num_diagonals);
    for (int i = 0; i < num_diagonals; i++)
        _diagonals[i].resize(_elastic_constraints.size() + _attachment_constraints.size());

    _solver.setNumDiagBlocks( _ordered_constraints.size() );
    _solver.setBandwidth(num_diagonals-1);

    // compute mass/inertia properties
    _m_total = _length * _cross_section->area() * _density;
    _m_node = _m_total / _num_nodes;
    _I_rot = _m_node / _cross_section->area() * Vec3r(_cross_section->Ix(), _cross_section->Iy(), _cross_section->Iz());
    _I_rot_inv = 1.0/_I_rot.array();

    // std::cout << "I_rot:\n" << _I_rot << std::endl;
    // std::cout << "I_rot_inv:\n" << _I_rot_inv << std::endl;

    // reserve space for constraints and constraint gradients
    _RHS_vec.conservativeResize(6* (_elastic_constraints.size() + _attachment_constraints.size()));
    _lambda.conservativeResize(6* (_elastic_constraints.size() + _attachment_constraints.size()));
    _dlam.conservativeResize(6* (_elastic_constraints.size() + _attachment_constraints.size()));
    _dx.conservativeResize(6*_num_nodes);
    
}

void XPBDRod::update(Real dt, Real g_accel)
{
    _lambda = VecXr::Zero(6*(_elastic_constraints.size() + _attachment_constraints.size()));
    _prev_nodes = _nodes;
    _inertialUpdate(dt, g_accel);

    for (int gi = 0; gi < 1; gi++)
    {
        // compute gradients (this will update the caches in the constraints so we won't have to recompute them)
        for (auto& c : _elastic_constraints)
        {
            c.gradient(true);
        }

        for (auto& c : _attachment_constraints)
        {
            c.gradient(true);
        }

        // compute RHS vector
        for (unsigned i = 0; i < _ordered_constraints.size(); i++)
        {    
            Vec6r C_i = std::visit([](const auto& constraint_ptr) -> Vec6r { return constraint_ptr->evaluate(); }, _ordered_constraints[i]);
            Vec6r alpha = std::visit([](const auto& constraint_ptr) -> Vec6r { return constraint_ptr->alpha(); }, _ordered_constraints[i]);

            _RHS_vec( Eigen::seqN(6*i, 6) ) = -C_i.array() - alpha.array() * _lambda( Eigen::seqN(6*i, 6) ).array() / (dt*dt);
        }

        // compute system matrix diagonals
        Vec6r inertia_mat_inv_node(1.0/_m_node, 1.0/_m_node, 1.0/_m_node, _I_rot_inv[0], _I_rot_inv[1], _I_rot_inv[2]); 
        for (unsigned i = 0; i < _ordered_constraints.size(); i++)
        {
            for (unsigned d = 0; d < _diagonals.size(); d++)
            {
                if (i + d >= _ordered_constraints.size())
                    break;
                
                _diagonals[d][i] = std::visit(ConstraintGradientProduct{inertia_mat_inv_node}, _ordered_constraints[i+d], _ordered_constraints[i]);
            }

            Vec6r alpha = std::visit([](const auto& constraint_ptr) -> Vec6r {return constraint_ptr->alpha(); }, _ordered_constraints[i]);
            _diagonals[0][i] += (alpha / (dt*dt)).asDiagonal();
        }
        


        
        // solve system
        _solver.solveInPlace(_diagonals, _RHS_vec, _dlam);

        // compute position update
        _dx = VecXr::Zero(6*_num_nodes);
        for (unsigned ci = 0; ci < _ordered_constraints.size(); ci++)
        {
            const Vec6r& dlam_ci = _dlam( Eigen::seqN(6*ci, 6) );
            std::visit(ComputePositionUpdateForConstraint{dlam_ci, &_dx, inertia_mat_inv_node}, _ordered_constraints[ci]);
        }

        // apply position updates
        _positionUpdate(); 

        _lambda += _dlam;
    }


    _velocityUpdate(dt);
}

int XPBDRod::_orderConstraints()
{
    // order constraints
    _ordered_constraints.resize(_elastic_constraints.size() + _attachment_constraints.size());

    int constraint_index = 0;
    int num_diagonals = 2;
    auto attachment_constraints_it = _attachment_constraints.begin();
    for (unsigned ei = 0; ei < _elastic_constraints.size(); ei++)
    {
        int num_constraints_affecting_node_i = (ei == 0) ? 1 : 2;
        while (attachment_constraints_it != _attachment_constraints.end() && attachment_constraints_it->nodeIndices()[0] == _elastic_constraints[ei].nodeIndices()[0])
        {
            _ordered_constraints[constraint_index++] = &(*attachment_constraints_it);
            attachment_constraints_it++;
            num_constraints_affecting_node_i++;
        }
        num_diagonals = std::max(num_constraints_affecting_node_i, num_diagonals);
        _ordered_constraints[constraint_index++] = &_elastic_constraints[ei];
    }

    int num_constraints_affecting_last_node = 1;
    while (attachment_constraints_it != _attachment_constraints.end())
    {
        _ordered_constraints[constraint_index++] = &(*attachment_constraints_it);
        attachment_constraints_it++;
        num_constraints_affecting_last_node++;
    }
    num_diagonals = std::max(num_diagonals, num_constraints_affecting_last_node);

    return num_diagonals;
}

void XPBDRod::_inertialUpdate(Real dt, Real g_accel)
{
    for (int i = 0; i < _num_nodes; i++)
    {
        auto& node = _nodes[i];

        // position inertial update
        Vec3r F_ext = _m_node * Vec3r(0,-g_accel,0);
        if (i==_num_nodes-1)
            F_ext = Vec3r(0,-10,0);
        node.position += dt*node.velocity + dt*dt/_m_node*F_ext;

        // orientation inertial update
        Vec3r T_ext = Vec3r(0,0,0);
        // if (i==_num_nodes - 1)
        //     T_ext = Vec3r(0,0,10000);
        Vec3r so3_update = dt*node.ang_velocity + dt*dt*_I_rot_inv.asDiagonal()*(T_ext - node.ang_velocity.cross(_I_rot.asDiagonal()*node.ang_velocity));
        node.orientation = Math::Plus_SO3(node.orientation, so3_update);
    }
}

void XPBDRod::_positionUpdate()
{
    for (int i = 0; i < _num_nodes; i++)
    {
        const Vec3r dp = _dx( Eigen::seqN(6*i,3) );
        const Vec3r dor = _dx( Eigen::seqN(6*i+3,3) );
        _nodes[i].position += dp;
        _nodes[i].orientation = Math::Plus_SO3(_nodes[i].orientation, dor);

        // std::cout << "New orientation " << i << ":\n" << _nodes[i].orientation << std::endl;
    }
}

void XPBDRod::_velocityUpdate(Real dt)
{
    for (unsigned i = 0; i < _nodes.size(); i++)
    {
        _nodes[i].velocity = (_nodes[i].position - _prev_nodes[i].position)/dt;
        // std::cout << "Orientation " << i << ":\n" << _nodes[i].orientation << std::endl;
        // std::cout << "Prev orientation " << i << ":\n" << _prev_nodes[i].orientation << std::endl;
        // std::cout << "Orientation velocity update Math::Minus_SO3:\n" << Math::Minus_SO3(_nodes[i].orientation, _prev_nodes[i].orientation) << std::endl;
        _nodes[i].ang_velocity = Math::Minus_SO3(_nodes[i].orientation, _prev_nodes[i].orientation)/dt;
    }
}

} // namespace Rod