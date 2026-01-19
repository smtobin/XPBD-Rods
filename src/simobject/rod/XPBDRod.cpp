#include "simobject/rod/XPBDRod.hpp"
#include "common/math.hpp"

#include "constraint/RodElasticConstraint.hpp"

#include <iostream>
#include <variant>

#include <Eigen/Cholesky>

namespace SimObject
{

std::vector<const OrientedParticle*> XPBDRod::particles() const
{
    std::vector<const OrientedParticle*> particles_vec;
    particles_vec.reserve(_nodes.size());
    for (const auto& node : _nodes)
    {
        particles_vec.push_back(&node);
    }

    return particles_vec;
}

void XPBDRod::setup()
{
    Real dl = _length / (_num_nodes - 1);

    Vec6r alpha_elastic = 1/dl * Vec6r( 1.0 / (_G * _cross_section->area()), 1.0 / (_G * _cross_section->area()), 1.0 / (_E * _cross_section->area()),
                                         1.0 / (_E * _cross_section->Ix()),   1.0 / (_E * _cross_section->Iy()),   1.0 / (_G * _cross_section->Iz()) );
    // create elastic constraints
    _elastic_constraints.reserve(_num_nodes-1);
    for (int i = 0; i < _num_nodes-1; i++)
    {
        _elastic_constraints.emplace_back(&_nodes[i], &_nodes[i+1], alpha_elastic);
    }

    _num_constraints = _elastic_constraints.size() + _attachment_constraints.size();

    // order constraints appropriately
    int num_diagonals = _orderConstraints();
    // resize diagonals
    _diagonals.resize(num_diagonals);
    for (int i = 0; i < num_diagonals; i++)
        _diagonals[i].resize(_num_constraints);

    _solver.setNumDiagBlocks(_num_constraints);
    _solver.setBandwidth(num_diagonals-1);


    if (_base_fixed)
        addOneSidedFixedJointConstraint(0, _nodes[0].position, _nodes[0].orientation);
    if (_tip_fixed)
        addOneSidedFixedJointConstraint(_num_nodes-1, _nodes.back().position, _nodes.back().orientation);

    // reserve space for constraints and constraint gradients
    _RHS_vec.conservativeResize(6*_num_constraints);
    _internal_lambda.conservativeResize(6*_num_constraints);
    _dlam.conservativeResize(6*_num_constraints);
    _dx.conservativeResize(6*_num_nodes);

    _prev_nodes = _nodes;
    
}

void XPBDRod::internalConstraintSolve(Real dt)
{
    _internal_lambda = VecXr::Zero(6*(_elastic_constraints.size() + _attachment_constraints.size()));
    // _prev_nodes = _nodes;
    // inertialUpdate(dt);

    for (int gi = 0; gi < 3; gi++)
    {
        // compute gradients (this will update the caches in the constraints so we won't have to recompute them)
        for (auto& c : _elastic_constraints)
        {
            c.gradient(true);
        }

        for (auto& [node_index, c] : _attachment_constraints)
        {
            c.gradient(true);
        }

        // compute RHS vector
        for (unsigned i = 0; i < _ordered_constraints.size(); i++)
        {    
            Vec6r C_i = std::visit([](const auto& constraint_ptr) -> Vec6r { return constraint_ptr->evaluate(); }, _ordered_constraints[i]);
            Vec6r alpha = std::visit([](const auto& constraint_ptr) -> Vec6r { return constraint_ptr->alpha(); }, _ordered_constraints[i]);

            _RHS_vec( Eigen::seqN(6*i, 6) ) = -C_i.array() - alpha.array() * _internal_lambda( Eigen::seqN(6*i, 6) ).array() / (dt*dt);
        }

        // compute system matrix diagonals
        Vec6r inertia_mat_inv_node(1.0/_m_node, 1.0/_m_node, 1.0/_m_node, _I_rot_inv[0], _I_rot_inv[1], _I_rot_inv[2]); 
        for (unsigned i = 0; i < _ordered_constraints.size(); i++)
        {
            for (unsigned d = 0; d < _diagonals.size(); d++)
            {
                if (i + d >= _ordered_constraints.size())
                    break;
                
                _diagonals[d][i] = std::visit(_ConstraintGradientProduct{inertia_mat_inv_node}, _ordered_constraints[i+d], _ordered_constraints[i]);
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
            std::visit(_ComputePositionUpdateForConstraint{&_nodes[0], dlam_ci, &_dx, inertia_mat_inv_node}, _ordered_constraints[ci]);
        }

        // apply position updates
        _positionUpdate(); 

        _internal_lambda += _dlam;
    }


    velocityUpdate(dt);
}

std::vector<ConstraintAndLambda> XPBDRod::internalConstraintsAndLambdas() const
{
    int lambda_index = 0;
    std::vector<ConstraintAndLambda> constraint_and_lambdas;
    constraint_and_lambdas.reserve(_num_constraints);
    for (const auto& constraint_ptr : _ordered_constraints)
    {
        std::visit([&](const auto& constraint) {
            XPBDConstraints_ConstPtrVariantType new_variant(constraint);
            const Real* lambda_ptr = _internal_lambda.data() + lambda_index;

            constraint_and_lambdas.emplace_back(new_variant, lambda_ptr);
        
            using ConstraintType = std::remove_cv_t<std::remove_pointer_t<std::decay_t<decltype(constraint)>>>;
            lambda_index += ConstraintType::ConstraintDim;
        }, constraint_ptr);
        
    }

    return constraint_and_lambdas;
}

Constraint::OneSidedFixedJointConstraint* XPBDRod::addOneSidedFixedJointConstraint(int node_index, const Vec3r& ref_position, const Mat3r& ref_orientation)
{
    std::multimap<int, Constraint::OneSidedFixedJointConstraint>::iterator it = 
        _attachment_constraints.emplace(std::make_pair(node_index, 
            Constraint::OneSidedFixedJointConstraint(ref_position, ref_orientation, &_nodes[node_index], Vec3r::Zero(), Mat3r::Identity())
        ));
    _num_constraints++;

    _RHS_vec.conservativeResize(6*_num_constraints);
    _internal_lambda.conservativeResize(6*_num_constraints);
    _dlam.conservativeResize(6*_num_constraints);

    // order constraints appropriately
    int num_diagonals = _orderConstraints();
    // resize diagonals
    _diagonals.resize(num_diagonals);
    for (int i = 0; i < num_diagonals; i++)
        _diagonals[i].resize(_num_constraints);

    // update the solver to reflect changes in the number of constraints and number of nonzero diagonals
    _solver.setNumDiagBlocks(_num_constraints);
    _solver.setBandwidth(num_diagonals-1);

    return &it->second;
}

bool XPBDRod::removeOneSidedFixedJointConstraint(int node_index, const Constraint::OneSidedFixedJointConstraint* ptr)
{
    using iterator = std::multimap<int, Constraint::OneSidedFixedJointConstraint>::iterator;
    bool constraint_removed = false;
    // if ptr is not nullptr, only erase the specific attachment constraint given
    if (ptr)
    {
        std::pair<iterator, iterator> iterpair = _attachment_constraints.equal_range(node_index);
    
        iterator it = iterpair.first;
        for (; it != iterpair.second; it++)
        {
            if (&it->second == ptr)
            {
                _attachment_constraints.erase(it);
                constraint_removed = true;
                _num_constraints--;
                break;
            }
        }
    }
    // otherwise, erase all attachment constraints for the given node index
    else
    {
        int num_removed = _attachment_constraints.erase(node_index);
        _num_constraints -= num_removed;
        constraint_removed = num_removed > 0;
    }

    // if we did in fact remove constraint(s), we need to reorder constraints
    if (constraint_removed)
    {
        // order constraints appropriately
        int num_diagonals = _orderConstraints();
        // resize diagonals
        _diagonals.resize(num_diagonals);
        for (int i = 0; i < num_diagonals; i++)
            _diagonals[i].resize(_num_constraints);

        // update the solver to reflect changes in the number of constraints and number of nonzero diagonals
        _solver.setNumDiagBlocks(_num_constraints);
        _solver.setBandwidth(num_diagonals-1);

        _RHS_vec.conservativeResize(6*_num_constraints);
        _internal_lambda.conservativeResize(6*_num_constraints);
        _dlam.conservativeResize(6*_num_constraints);
    }

    return constraint_removed;
    
}

int XPBDRod::_orderConstraints()
{
    _ordered_constraints.resize(_num_constraints);

    // tracks where to insert the next constraint in the _ordered_constraints vector
    int constraint_index = 0;   
    // tracks the number of nonzero diagonals in the lambda system matrix
    // by default, this is 2 (i.e. a tridiagonal system), but may be more if there are additional constraints applied to middle nodes
    int num_diagonals = 2;
    // an iterator for going through attachment constraints       
    auto attachment_constraints_it = _attachment_constraints.begin();

    for (unsigned ei = 0; ei < _elastic_constraints.size(); ei++)
    {
        // the number of elastic constraints affecting node "ei" - only 1 elastic constraint affects the first node, but all middle nodes have 2 elastic constraints
        int num_constraints_affecting_node_ei = (ei == 0) ? 1 : 2;

        // check if there are any attachment constraints that affect node ei
        // if there are, we need to insert them before the elastic constraint to maintain the proper ordering for a block banded system matrix
        while (attachment_constraints_it != _attachment_constraints.end() && 
            attachment_constraints_it->second.particles()[0] == _elastic_constraints[ei].particles()[0])
        {
            // add the attachment constraint to the _ordered_constraints vector
            _ordered_constraints[constraint_index++] = &attachment_constraints_it->second;
            attachment_constraints_it++;
            // increment the number of constraints affecting node ei (since this constraint affects node ei)
            num_constraints_affecting_node_ei++;
        }
        // the number of diagonals in the lambda system matrix = the maximum number of constraints affecting a single node
        num_diagonals = std::max(num_constraints_affecting_node_ei, num_diagonals);

        // after adding any attachment constraints that affect node ei, add the elastic constraint that affects nodes ei and ei+1
        _ordered_constraints[constraint_index++] = &_elastic_constraints[ei];
    }


    // do a final check for attachment constraints affecting the last node
    // these should come after the last elastic constraint

    // same as before, keep track of the number of constraints affecting the last node - in this case only 1 elastic constraint affects the last node
    int num_constraints_affecting_last_node = 1;
    // add any attachment constraints affecting the last node
    while (attachment_constraints_it != _attachment_constraints.end())
    {
        _ordered_constraints[constraint_index++] = &attachment_constraints_it->second;
        attachment_constraints_it++;
        num_constraints_affecting_last_node++;
    }
    // the number of diagonals in the lambda system matrix = the maximum number of constraints affecting a single node
    num_diagonals = std::max(num_diagonals, num_constraints_affecting_last_node);

    return num_diagonals;
}

void XPBDRod::inertialUpdate(Real dt)
{
    _prev_nodes = _nodes;

    for (int i = 0; i < _num_nodes; i++)
    {
        auto& node = _nodes[i];

        Vec3r F_ext = node.mass * Vec3r(0,-G_ACCEL,0);
        Vec3r T_ext = Vec3r(0,0,0);
        node.inertialUpdate(dt, F_ext, T_ext);
    }
}

void XPBDRod::_positionUpdate()
{
    for (int i = 0; i < _num_nodes; i++)
    {
        const Vec3r dp = _dx( Eigen::seqN(6*i,3) );
        const Vec3r dor = _dx( Eigen::seqN(6*i+3,3) );
        _nodes[i].positionUpdate(dp, dor);
    }
}

void XPBDRod::velocityUpdate(Real dt)
{
    for (unsigned i = 0; i < _nodes.size(); i++)
    {
        _nodes[i].velocityUpdate(dt, _prev_nodes[i].position, _prev_nodes[i].orientation);
    }
}

} // namespace SimObject