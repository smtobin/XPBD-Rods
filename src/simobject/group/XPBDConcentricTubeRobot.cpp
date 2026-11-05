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

    _inner_tube_config = config.innerTubeConfig();
    _outer_tube_config = config.outerTubeConfig();

    // initialize the tube positions such that their tip is at the base position
    Vec3r inner_tube_pos = _base_position - _base_orientation.col(2)*config.innerTubeConfig().length();
    Vec3r outer_tube_pos = _base_position - _base_orientation.col(2)*config.outerTubeConfig().length();
    _inner_tube_config.setInitialPosition(inner_tube_pos);
    _outer_tube_config.setInitialPosition(outer_tube_pos);

    // initial tube rotations to align with base orientation
    _inner_tube_config.setInitialRotation(config.initialRotation());
    _outer_tube_config.setInitialRotation(config.initialRotation());

    _inner_tube_translation = 0;
    _inner_tube_rotation = 0;
    _outer_tube_translation = 0;
    _outer_tube_rotation = 0;
}

void XPBDConcentricTubeRobot::setup()
{
    _objects.template reserve<XPBDRod_<RodElement<1>>>(2);
    _outer_rod = &_addObject<XPBDRod_<RodElement<1>>>(_outer_tube_config);
    _inner_rod = &_addObject<XPBDRod_<RodElement<1>>>(_inner_tube_config);

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
    const XPBDConstraints_Container& outer_rod_all_constraints = _outer_rod->internalConstraints();
    const XPBDConstraints_Container& inner_rod_all_constraints = _inner_rod->internalConstraints();
    using ElasticConstraintType = XPBDRod_<RodElement<1>>::ElasticConstraintType;
    const std::vector<ElasticConstraintType>& outer_rod_constraints = outer_rod_all_constraints.get<ElasticConstraintType>();
    const std::vector<ElasticConstraintType>& inner_rod_constraints = inner_rod_all_constraints.get<ElasticConstraintType>();

    _alpha = VecXr::Zero(6*outer_rod_constraints.size() + 6*inner_rod_constraints.size());
    int constraint_index = 0;
    for (const auto& constraint : outer_rod_constraints)
    {
        _alpha.block<6,1>(6*constraint_index++, 0) = constraint.alpha();
    }
    for (const auto& constraint : inner_rod_constraints)
    {
        _alpha.block<6,1>(6*constraint_index++, 0) = constraint.alpha();
    }


    // add fixed base constraint for outer rod
    _addInternalConstraint<Constraint::OneSidedFixedJointConstraint>(
        _outer_rod->nodes().front().position, _outer_rod->nodes().front().orientation,
         &_outer_rod->nodes().front(), Vec3r::Zero(), Mat3r::Identity());

    // add fixed base constraint for inner rod
    _addInternalConstraint<Constraint::OneSidedFixedJointConstraint>(
        _inner_rod->nodes().front().position, _inner_rod->nodes().front().orientation,
         &_inner_rod->nodes().front(), Vec3r::Zero(), Mat3r::Identity());

    _outer_rod_base_constraint = &_internal_constraints.template get<Constraint::OneSidedFixedJointConstraint>()[0];
    _inner_rod_base_constraint = &_internal_constraints.template get<Constraint::OneSidedFixedJointConstraint>()[1];

    // create the initial point-on-line constraints
    _updateConcentricityConstraints();
    
}

void XPBDConcentricTubeRobot::setInnerTubeTranslation(Real it_trans)
{
    Vec3r inner_tube_pos = _base_position - _base_orientation.col(2)*(_inner_tube_config.length() - it_trans);
    _inner_rod_base_constraint->setReferencePosition(inner_tube_pos);
    _inner_tube_translation = it_trans;
}

void XPBDConcentricTubeRobot::setInnerTubeRotation(Real it_rot)
{
    Mat3r inner_tube_rot = _base_orientation * Math::RotMatFromXYZEulerAngles(Vec3r(0, 0, it_rot));
    _inner_rod_base_constraint->setReferenceOrientation(inner_tube_rot);
    _inner_tube_rotation = it_rot;
}

void XPBDConcentricTubeRobot::setOuterTubeTranslation(Real ot_trans)
{
    Vec3r outer_tube_pos = _base_position - _base_orientation.col(2)*(_outer_tube_config.length() - ot_trans);
    _outer_rod_base_constraint->setReferencePosition(outer_tube_pos);
    _outer_tube_translation = ot_trans;
}

void XPBDConcentricTubeRobot::setOuterTubeRotation(Real ot_rot)
{
    Mat3r outer_tube_rot = _base_orientation * Math::RotMatFromXYZEulerAngles(Vec3r(0, 0, ot_rot));
    _outer_rod_base_constraint->setReferenceOrientation(outer_tube_rot);
    _outer_tube_rotation = ot_rot;
}


void XPBDConcentricTubeRobot::_updateConcentricityConstraints()
{
    // for now, clear the point-on-line constraints every time
    _internal_constraints.template clear<Constraint::OneSidedPointLineConstraint>();
    _internal_constraints.template clear<Constraint::PointLineConstraint>();

    std::vector<OrientedParticle>& outer_nodes = _outer_rod->nodes();
    std::vector<OrientedParticle>& inner_nodes = _inner_rod->nodes();

    // go through each outer node and determine if it is inside the rigid sheath
    for (unsigned outer_node_index = 1; outer_node_index < outer_nodes.size(); outer_node_index++)
    {
        OrientedParticle& outer_node = outer_nodes[outer_node_index];
        Real s = Real(outer_node_index) / (outer_nodes.size() - 1) * _outer_tube_config.length();
        if (s + _outer_tube_translation <= _outer_tube_config.length())
        {
            // this node is inside the sheath
            _addInternalConstraint<Constraint::OneSidedPointLineConstraint>(&outer_node, _base_position, _base_position + _base_orientation.col(2)*_outer_tube_config.length());
            continue;
        }
    }

    // go through each inner node and project it onto a line segment in the outer tube
    // std::cout <<"\nFinding concentricity correspondences..." << std::endl;
    for (unsigned inner_node_index = 1; inner_node_index < inner_nodes.size(); inner_node_index++)
    {
        OrientedParticle& inner_node = inner_nodes[inner_node_index];

        Real s = Real(inner_node_index) / (inner_nodes.size() - 1) * _inner_tube_config.length();
        if (s + _inner_tube_translation <= _inner_tube_config.length())
        {
            // this node is inside the sheath
            _addInternalConstraint<Constraint::OneSidedPointLineConstraint>(&inner_node, _base_position, _base_position + _base_orientation.col(2)*_inner_tube_config.length());
            continue;
        }

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

    const XPBDConstraints_Container& outer_rod_all_constraints = _outer_rod->internalConstraints();
    const XPBDConstraints_Container& inner_rod_all_constraints = _inner_rod->internalConstraints();

    using ElasticConstraintType = XPBDRod_<RodElement<1>>::ElasticConstraintType;
    const std::vector<ElasticConstraintType>& outer_rod_constraints = outer_rod_all_constraints.get<ElasticConstraintType>();
    // std::cout << "Num outer rod constraints: " << outer_rod_constraints.size() << std::endl;
    const std::vector<ElasticConstraintType>& inner_rod_constraints = inner_rod_all_constraints.get<ElasticConstraintType>();
    // std::cout << "Num inner rod constraints: " << inner_rod_constraints.size() << std::endl;
    const std::vector<Constraint::OneSidedFixedJointConstraint>& fixed_base_constraints = _internal_constraints.template get<Constraint::OneSidedFixedJointConstraint>();

    const std::vector<Constraint::PointLineConstraint>& concentricity_constraints = _internal_constraints.template get<Constraint::PointLineConstraint>();
    const std::vector<Constraint::OneSidedPointLineConstraint>& sheath_constraints = _internal_constraints.template get<Constraint::OneSidedPointLineConstraint>();

    std::cout << "Number of concentricity constraints: " << concentricity_constraints.size() << std::endl;
    std::cout << "Number of sheath constraints: " << sheath_constraints.size() << std::endl;

    // calculate number of constraints
    int num_constraints = fixed_base_constraints.size()*Constraint::OneSidedFixedJointConstraint::ConstraintDim + 
                          outer_rod_constraints.size()*ElasticConstraintType::ConstraintDim +
                          inner_rod_constraints.size()*ElasticConstraintType::ConstraintDim + 
                          concentricity_constraints.size()*Constraint::PointLineConstraint::ConstraintDim +
                          sheath_constraints.size()*Constraint::OneSidedPointLineConstraint::ConstraintDim;
    
    _RHS_vec.conservativeResize(num_constraints, 1);
    _alpha.conservativeResize(num_constraints, 1);
    _delC_mat = MatXr::Zero(num_constraints, _outer_rod->nodes().size()*6 + _inner_rod->nodes().size()*6);
    _LHS_mat.conservativeResize(num_constraints, num_constraints );
    // start assembling global system
    int row_offset = 0;
    // fix the outer tube base
    for (const auto& constraint : fixed_base_constraints)
    {
        Vec6r C = constraint.evaluate();
        _RHS_vec.block<6,1>(row_offset,0) = -C;

        // std::cout << "Fixed base constraint: " << C.transpose() << std::endl;

        Constraint::OneSidedFixedJointConstraint::GradientMatType grad = constraint.gradient();
        int pos_index = _particle_ptr_to_index.at(constraint.orientedParticles()[0]);
        _delC_mat.block<6,6>(row_offset, pos_index*6) = grad;

        _alpha.block<6,1>(row_offset,0) = constraint.alpha();

        row_offset += Constraint::OneSidedFixedJointConstraint::ConstraintDim;

    } 

    for (const auto& constraint : outer_rod_constraints)
    {
        Vec6r C = constraint.evaluate();
        _RHS_vec.block<6,1>(row_offset, 0) = -C;

        // std::cout << "Outer rod constraint: " << C.transpose() << std::endl;

        ElasticConstraintType::GradientMatType grad = constraint.gradient();
        int pos1_index = _particle_ptr_to_index.at(constraint.orientedParticles()[0]);
        int pos2_index = _particle_ptr_to_index.at(constraint.orientedParticles()[1]);
        _delC_mat.block<6,6>(row_offset, pos1_index*6) = grad.block<6,6>(0,0);
        _delC_mat.block<6,6>(row_offset, pos2_index*6) = grad.block<6,6>(0,6);

        _alpha.block<6,1>(row_offset,0) = constraint.alpha();

        row_offset += ElasticConstraintType::ConstraintDim;
    }

    for (const auto& constraint : inner_rod_constraints)
    {
        Vec6r C = constraint.evaluate();
        _RHS_vec.block<6,1>(row_offset, 0) = -C;

        // std::cout << "Inner rod constraint: " << C.transpose() << std::endl;

        ElasticConstraintType::GradientMatType grad = constraint.gradient();
        int pos1_index = _particle_ptr_to_index.at(constraint.orientedParticles()[0]);
        int pos2_index = _particle_ptr_to_index.at(constraint.orientedParticles()[1]);
        _delC_mat.block<6,6>(row_offset, pos1_index*6) = grad.block<6,6>(0,0);
        _delC_mat.block<6,6>(row_offset, pos2_index*6) = grad.block<6,6>(0,6);

        _alpha.block<6,1>(row_offset,0) = constraint.alpha();

        row_offset += ElasticConstraintType::ConstraintDim;
    }

    for (const auto& constraint : concentricity_constraints)
    {
        Real C = constraint.evaluate()[0];
        _RHS_vec[row_offset] = -C;

        // std::cout << "Point on line constraint: " << C << std::endl;

        Constraint::PointLineConstraint::GradientMatType grad = constraint.gradient();
        int pos1_index = _particle_ptr_to_index.at(constraint.orientedParticles()[0]);
        int pos2_index = _particle_ptr_to_index.at(constraint.orientedParticles()[1]);
        int pos3_index = _particle_ptr_to_index.at(constraint.orientedParticles()[2]);
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

    for (const auto& constraint : sheath_constraints)
    {
        Real C = constraint.evaluate()[0];
        _RHS_vec[row_offset] = -C;

        // std::cout << "Point on line constraint: " << C << std::endl;

        Constraint::OneSidedPointLineConstraint::GradientMatType grad = constraint.gradient();
        int pos1_index = _particle_ptr_to_index.at(constraint.orientedParticles()[0]);
        _delC_mat.block<1,6>(row_offset, pos1_index*6) = grad.block<1,6>(0,0);

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

        row_offset += Constraint::OneSidedPointLineConstraint::ConstraintDim;
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

    const XPBDConstraints_Container& outer_rod_all_constraints = _outer_rod->internalConstraints();
    const XPBDConstraints_Container& inner_rod_all_constraints = _inner_rod->internalConstraints();

    using ElasticConstraintType = XPBDRod_<RodElement<1>>::ElasticConstraintType;
    const std::vector<ElasticConstraintType>& outer_rod_constraints = outer_rod_all_constraints.get<ElasticConstraintType>();
    // std::cout << "Num outer rod constraints: " << outer_rod_constraints.size() << std::endl;
    const std::vector<ElasticConstraintType>& inner_rod_constraints = inner_rod_all_constraints.get<ElasticConstraintType>();
    // std::cout << "Num inner rod constraints: " << inner_rod_constraints.size() << std::endl;
    const std::vector<Constraint::OneSidedFixedJointConstraint>& fixed_base_constraints = _internal_constraints.template get<Constraint::OneSidedFixedJointConstraint>();

    // calculate number of constraints
    int num_constraints = fixed_base_constraints.size()*Constraint::OneSidedFixedJointConstraint::ConstraintDim + 
                          outer_rod_constraints.size()*ElasticConstraintType::ConstraintDim +
                          inner_rod_constraints.size()*ElasticConstraintType::ConstraintDim + 
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
        lambda_index += ElasticConstraintType::ConstraintDim;
    }

    for (const auto& constraint : inner_rod_constraints)
    {
        XPBDConstraints_ConstPtrVariantType var(&constraint);
        const Real* lambda_ptr = _internal_lambda.data() + lambda_index;
        constraints_and_lambdas.emplace_back(var, lambda_ptr);
        lambda_index += ElasticConstraintType::ConstraintDim;
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

void XPBDConcentricTubeRobot::velocityUpdate(Real dt)
{
    XPBDObjectGroup_Base::velocityUpdate(dt);

    // create some sliding motion
    setInnerTubeTranslation(std::sin(0.5*_t));

    _t += dt;
}

} // namespace SimObject