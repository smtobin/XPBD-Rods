#include "simobject/group/XPBDConcentricTubeRobot.hpp"

#include "config/RodConfig.hpp"

#include "constraint/PointLineConstraint.hpp"

#include <Eigen/Cholesky>

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
        1, 0.1, 20, 1000, 1e6, 0.3
    );

    SimObject::CircleCrossSection outer_cross_section(0.1/2.0, 10);

    Config::RodConfig inner_rod_config(
        "inner", _base_position, Vec3r(0,0,0), Vec3r(0,0,0), Vec3r(0,0,0), true, false,
        1.5, 0.08, 20, 1000, 1e6, 0.3
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


    // add fixed base constraint for outer rod
    _addInternalConstraint<Constraint::OneSidedFixedJointConstraint>(Vec3r::Zero(), Mat3r::Identity(), &_outer_rod->nodes().front(), Vec3r::Zero(), Mat3r::Identity());

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
    // std::cout <<"\nFinding concentricity correspondences..." << std::endl;
    for (unsigned inner_node_index = 0; inner_node_index < inner_nodes.size(); inner_node_index++)
    {
        OrientedParticle& inner_node = inner_nodes[inner_node_index];

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

        // std::cout << "  Inner node index: " << inner_node_index << " Outer segment: " << outer_node_index << ", " << outer_node_index+1 << std::endl;
        // create a point-on-line constraint between the inner tube node and the outer tube segment
        _addInternalConstraint<Constraint::PointLineConstraint>(&inner_node, &outer_nodes[outer_node_index], &outer_nodes[outer_node_index+1]);
    }
}

void XPBDConcentricTubeRobot::internalConstraintSolve(Real dt)
{

    _updateConcentricityConstraints();

    const std::vector<Constraint::RodElasticConstraint>& outer_rod_constraints = _outer_rod->rodConstraints();
    // std::cout << "Num outer rod constraints: " << outer_rod_constraints.size() << std::endl;
    const std::vector<Constraint::RodElasticConstraint>& inner_rod_constraints = _inner_rod->rodConstraints();
    // std::cout << "Num inner rod constraints: " << inner_rod_constraints.size() << std::endl;
    const std::vector<Constraint::OneSidedFixedJointConstraint>& fixed_base_constraints = _internal_constraints.template get<Constraint::OneSidedFixedJointConstraint>();

    // calculate number of constraints
    int num_constraints = fixed_base_constraints.size()*Constraint::OneSidedFixedJointConstraint::ConstraintDim + 
                          outer_rod_constraints.size()*Constraint::RodElasticConstraint::ConstraintDim +
                          inner_rod_constraints.size()*Constraint::RodElasticConstraint::ConstraintDim + 
                          _internal_constraints.template get<Constraint::PointLineConstraint>().size()*Constraint::PointLineConstraint::ConstraintDim;
    
    _RHS_vec.conservativeResize(num_constraints, 1);
    _alpha.conservativeResize(num_constraints, 1);
    _delC_mat.conservativeResize(num_constraints, _outer_rod->nodes().size()*6 + _inner_rod->nodes().size()*6);
    _LHS_mat.conservativeResize(_outer_rod->nodes().size()*6 + _inner_rod->nodes().size()*6, _outer_rod->nodes().size()*6 + _inner_rod->nodes().size()*6 );
    // start assembling global system
    int row_offset = 0;
    // fix the outer tube base
    for (const auto& constraint : fixed_base_constraints)
    {
        Vec6r C = constraint.evaluate();
        _RHS_vec.block<6,1>(row_offset,0) = -C;

        // std::cout << "Fixed base constraint: " << C.transpose() << std::endl;

        Constraint::OneSidedFixedJointConstraint::GradientMatType grad = constraint.gradient();
        int pos_index = _particle_ptr_to_index.at(constraint.particles()[0]);
        _delC_mat.block<6,6>(row_offset, pos_index*6) = grad;

        _alpha.block<6,1>(row_offset,0) = constraint.alpha();

        row_offset += Constraint::OneSidedFixedJointConstraint::ConstraintDim;

    } 

    for (const auto& constraint : outer_rod_constraints)
    {
        Vec6r C = constraint.evaluate();
        _RHS_vec.block<6,1>(row_offset, 0) = -C;

        // std::cout << "Outer rod constraint: " << C.transpose() << std::endl;

        Constraint::RodElasticConstraint::GradientMatType grad = constraint.gradient();
        int pos1_index = _particle_ptr_to_index.at(constraint.particles()[0]);
        int pos2_index = _particle_ptr_to_index.at(constraint.particles()[1]);
        _delC_mat.block<6,6>(row_offset, pos1_index*6) = grad.block<6,6>(0,0);
        _delC_mat.block<6,6>(row_offset, pos2_index*6) = grad.block<6,6>(0,6);

        _alpha.block<6,1>(row_offset,0) = constraint.alpha();

        row_offset += Constraint::RodElasticConstraint::ConstraintDim;
    }

    for (const auto& constraint : inner_rod_constraints)
    {
        Vec6r C = constraint.evaluate();
        _RHS_vec.block<6,1>(row_offset, 0) = -C;

        // std::cout << "Inner rod constraint: " << C.transpose() << std::endl;

        Constraint::RodElasticConstraint::GradientMatType grad = constraint.gradient();
        int pos1_index = _particle_ptr_to_index.at(constraint.particles()[0]);
        int pos2_index = _particle_ptr_to_index.at(constraint.particles()[1]);
        _delC_mat.block<6,6>(row_offset, pos1_index*6) = grad.block<6,6>(0,0);
        _delC_mat.block<6,6>(row_offset, pos2_index*6) = grad.block<6,6>(0,6);

        _alpha.block<6,1>(row_offset,0) = constraint.alpha();

        row_offset += Constraint::RodElasticConstraint::ConstraintDim;
    }

    for (const auto& constraint : _internal_constraints.template get<Constraint::PointLineConstraint>())
    {
        Real C = constraint.evaluate()[0];
        _RHS_vec[row_offset] = -C;

        // std::cout << "Point on line constraint: " << C << std::endl;

        Constraint::PointLineConstraint::GradientMatType grad = constraint.gradient();
        int pos1_index = _particle_ptr_to_index.at(constraint.particles()[0]);
        int pos2_index = _particle_ptr_to_index.at(constraint.particles()[1]);
        int pos3_index = _particle_ptr_to_index.at(constraint.particles()[2]);
        _delC_mat.block<1,6>(row_offset, pos1_index*6) = grad.block<1,6>(0,0);
        _delC_mat.block<1,6>(row_offset, pos2_index*6) = grad.block<1,6>(0,6);
        _delC_mat.block<1,6>(row_offset, pos3_index*6) = grad.block<1,6>(0,12);

        /** TODO: when the point lies on the line (i.e. C=0), then the constraint gradient is undefined.
         * For now, we set the entire constraint gradient to all zeros. But, this creates a singular LHS matrix since
         * alpha is also 0. 
         * 
         * Option 1: detect this and remove these constraints from the system
         * Option 2 (used below): change alpha temporarily to be nonzero so that the matrix is not singular. Doesn't matter anyway since the RHS is 0.
         */
        if (abs(C) < 1e-10)
            _alpha[row_offset] = 1e-3;
        else
            _alpha[row_offset] = constraint.alpha()[0];

        row_offset += Constraint::PointLineConstraint::ConstraintDim;
    }

    // compute LHS
    // std::cout << "Minv: " << _M_inv.transpose() << std::endl;
    // std::cout << "_delC_mat: \n" << _delC_mat << std::endl;
    // std::cout << "_delC size: " << _delC_mat.rows() << " x " << _delC_mat.cols() << "  _M_inv size: " << _M_inv.size() << std::endl;
    _LHS_mat = _delC_mat * _M_inv.asDiagonal() * _delC_mat.transpose();
    // std::cout << "LHS mat: " << _LHS_mat << std::endl;
    // std::cout << "_LHS_mat size: " << _LHS_mat.rows() << " x " << _LHS_mat.cols() << std::endl;
    // std::cout << "_alpha size: " << _alpha.size() << std::endl;
    _LHS_mat.diagonal() += _alpha/(dt*dt);

    // std::cout << "LHS mat: " << _LHS_mat << std::endl;
    // std::cout << "RHS vec: " << _RHS_vec.transpose() << std::endl;
    // std::cout << "alpha vec: " << _alpha.transpose() << std::endl;
    // std::cout << "dt : " << dt << std::endl;

    // std::cout << "Number of NaNs in LHS mat: " << _LHS_mat.array().isNaN().sum() << std::endl;

    Eigen::LLT<Eigen::MatrixXd> llt(_LHS_mat);
    // solve and compute position update
    // _dlam = _LHS_mat.llt().solve(_RHS_vec);
    _dlam = llt.solve(_RHS_vec);
    _internal_lambda = _dlam;
    // std::cout << "dlam: " << _dlam.transpose() << std::endl;
    // std::cout << "Number of NaNs in dlam: " << _dlam.array().isNaN().sum() << std::endl;
    // std::cout << "_dlam size: " << _dlam.size() << std::endl;
    _dx = _M_inv.asDiagonal() * _delC_mat.transpose() * _dlam;
    // std::cout << "Position update: " << _dx.transpose() << std::endl;
    // std::cout << "_dx size: " << _dx.size() << std::endl;

    // update positions
    for (unsigned i = 0; i < _outer_rod->nodes().size(); i++)
    {
        const Vec3r dp = _dx( Eigen::seqN(6*i,3) );
        const Vec3r dor = _dx( Eigen::seqN(6*i+3,3) );
        _outer_rod->nodes()[i].positionUpdate(dp, dor);
    }

    for (unsigned i = 0; i < _inner_rod->nodes().size(); i++)
    {
        const Vec3r dp = _dx( Eigen::seqN(6*(i+_outer_rod->nodes().size()),3) );
        const Vec3r dor = _dx( Eigen::seqN(6*(i+_outer_rod->nodes().size())+3,3) );
        _inner_rod->nodes()[i].positionUpdate(dp, dor);
    }
}

std::vector<ConstraintAndLambda> XPBDConcentricTubeRobot::internalConstraintsAndLambdas() const
{
    int lambda_index = 0;

    const std::vector<Constraint::RodElasticConstraint>& outer_rod_constraints = _outer_rod->rodConstraints();
    const std::vector<Constraint::RodElasticConstraint>& inner_rod_constraints = _inner_rod->rodConstraints();
    const std::vector<Constraint::OneSidedFixedJointConstraint>& fixed_base_constraints = _internal_constraints.template get<Constraint::OneSidedFixedJointConstraint>();

    // calculate number of constraints
    int num_constraints = fixed_base_constraints.size()*Constraint::OneSidedFixedJointConstraint::ConstraintDim + 
                          outer_rod_constraints.size()*Constraint::RodElasticConstraint::ConstraintDim +
                          inner_rod_constraints.size()*Constraint::RodElasticConstraint::ConstraintDim + 
                          _internal_constraints.template get<Constraint::PointLineConstraint>().size()*Constraint::PointLineConstraint::ConstraintDim;
    
    std::vector<ConstraintAndLambda> constraints_and_lambdas;
    constraints_and_lambdas.reserve(num_constraints);

    // currently, constraints are ordered in the internal global system solve as follows:
    //  1. fixed base constraints
    //  2. outer rod elastic constraints
    //  3. inner rod elastic constraints
    //  4. point on line constraints between rods
    // so, assemble the constraints and lambdas in that order

    // fix the outer tube base
    for (const auto& constraint : fixed_base_constraints)
    {
        XPBDConstraints_ConstPtrVariantType var(&constraint);
        const Real* lambda_ptr = _internal_lambda.data() + lambda_index;
        constraints_and_lambdas.emplace_back(var, lambda_ptr);
        lambda_index += Constraint::OneSidedFixedJointConstraint::ConstraintDim;
    } 

    for (const auto& constraint : outer_rod_constraints)
    {
        XPBDConstraints_ConstPtrVariantType var(&constraint);
        const Real* lambda_ptr = _internal_lambda.data() + lambda_index;
        constraints_and_lambdas.emplace_back(var, lambda_ptr);
        lambda_index += Constraint::RodElasticConstraint::ConstraintDim;
    }

    for (const auto& constraint : inner_rod_constraints)
    {
        XPBDConstraints_ConstPtrVariantType var(&constraint);
        const Real* lambda_ptr = _internal_lambda.data() + lambda_index;
        constraints_and_lambdas.emplace_back(var, lambda_ptr);
        lambda_index += Constraint::RodElasticConstraint::ConstraintDim;
    }

    for (const auto& constraint : _internal_constraints.template get<Constraint::PointLineConstraint>())
    {
        XPBDConstraints_ConstPtrVariantType var(&constraint);
        const Real* lambda_ptr = _internal_lambda.data() + lambda_index;
        constraints_and_lambdas.emplace_back(var, lambda_ptr);
        lambda_index += Constraint::PointLineConstraint::ConstraintDim;
    }

    return constraints_and_lambdas;
}

} // namespace SimObject