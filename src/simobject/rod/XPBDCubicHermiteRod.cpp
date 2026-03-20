#include "simobject/rod/XPBDCubicHermiteRod.hpp"

#include "common/GaussQuadratureHelper.hpp"

#include <Eigen/Cholesky>

template<typename T>
struct base_type { using type = T; };

template<typename T>
struct base_type<T*> : base_type<T> {};

template<typename T>
struct base_type<T&> : base_type<T> {};

template<typename T>
struct base_type<T&&> : base_type<T> {};

template<typename T>
struct base_type<const T> : base_type<T> {};

template<typename T>
struct base_type<volatile T> : base_type<T> {};

template<typename T>
using base_type_t = typename base_type<T>::type;

namespace SimObject
{

XPBDCubicHermiteRod::XPBDCubicHermiteRod(const Config::RodConfig& config)
    : XPBDRod_<CubicHermiteRodElement>(config)
{
}

void XPBDCubicHermiteRod::setup()
{
    /** Initialize derivative DOF */
    _dp_DOF.resize(_num_nodes);
    _dR_DOF.resize(_num_nodes);

    // arc-length derivative of local position should be initially h*(0,0,1)
    // so use the base rotation to transform that into the global frame
    // (premultiply by h, the element rest length, to nondimensionalize the derivative DOF)
    for (int i = 0; i < _num_nodes; i++)
    {
        _dp_DOF[i].position = _nodes[0].orientation * _element_rest_length * Vec3r(0,0,1);
        _dp_DOF[i].velocity = Vec3r::Zero();
        _dp_DOF[i].prev_position = _dp_DOF[i].position;
        _dp_DOF[i].mass = Vec3r::Zero(); // this will be done later
    }

    // arc-length derivative of local rotation should be initially (0,0,0)
    //  (unless there is a prescribed precurvature of the rod)
    for (int i = 0; i < _num_nodes; i++)
    {
        _dR_DOF[i].position = Vec3r::Zero();
        _dR_DOF[i].velocity = Vec3r::Zero();
        _dR_DOF[i].prev_position = _dR_DOF[i].position;
        _dR_DOF[i].mass = Vec3r::Zero(); // this will be done later
    }


    /** Assign masses */

    // lumped mass proportions
    auto lumped_masses = ElementType::lumpedMasses();

    for (int i = 0; i < _num_elements; i++)
    {
        // total element mass
        Real total_mass = _element_rest_length * _area * _density;
        Vec3r total_rot_inertia = _density * _element_rest_length * Vec3r(_Ix, _Ix, _Iz);

        // DOF ordering is [p, hp', R, hR']
        _nodes[i].mass += total_mass * lumped_masses[0];
        _nodes[i].Ib += total_rot_inertia * lumped_masses[0];
        _nodes[i+1].mass += total_mass * lumped_masses[2];
        _nodes[i+1].Ib += total_rot_inertia * lumped_masses[2];

        _dp_DOF[i].mass += Vec3r::Ones() * total_mass * lumped_masses[1];
        _dp_DOF[i+1].mass += Vec3r::Ones() * total_mass * lumped_masses[3];

        _dR_DOF[i].mass += total_rot_inertia * lumped_masses[1];
        _dR_DOF[i+1].mass += total_rot_inertia * lumped_masses[3];
    }

    // compute inverse inertias
    _node_inverse_inertias.resize(_num_nodes);
    for (int i = 0; i < _num_nodes; i++)
    {
        _node_inverse_inertias[i] = Vec6r(1.0/_nodes[i].mass, 1.0/_nodes[i].mass, 1.0/_nodes[i].mass, 1.0/_nodes[i].Ib[0], 1.0/_nodes[i].Ib[1], 1.0/_nodes[i].Ib[2]);
    }

    /** Create elements */
    for (int i = 0; i < _num_elements; i++)
    {
        const std::array<OrientedParticle*, 2> element_nodes = {&_nodes[i], &_nodes[i+1]};
        const std::array<Particle*, 2> element_dp = {&_dp_DOF[i], &_dp_DOF[i+1]};
        const std::array<Particle*, 2> element_dR = {&_dR_DOF[i], &_dR_DOF[i+1]};

        _elements.emplace_back(element_nodes, element_dp, element_dR, _element_rest_length);
    }

    /** Create collision segments */
    // divide the rod up into collision segments that are at least as long as the diameter of the rod
    // this prevents segments from within the rod fighting with each other in collision detection
    // and makes collision detection a lot faster if we downsample
    int num_segments_per_dia = static_cast<int>(2*_radius / _element_rest_length) + 1;    // round up
    int remainder = (_num_nodes - 1) % num_segments_per_dia;
    int num_collision_segments = (_num_nodes - 1) / num_segments_per_dia;
    _collision_segments.reserve(num_collision_segments);

    int cur_index = 0;
    for (int i = 0; i < num_collision_segments; i++)
    {
        int size = num_segments_per_dia;
        if (remainder > 0)
        {
            size++;
            remainder--;
        }

        std::vector<RodElement_Base*> seg_elements;
        for (int k = 0; k < size; k++)
        {
            seg_elements.push_back(&_elements[cur_index + k]);
        }

        _collision_segments.emplace_back(seg_elements, _radius, _mu_s, _mu_d);
        cur_index += size;
    }

    /** Create elastic constraints */
    // get a reference to the elastic constraints
    std::vector<ElasticConstraintType>& elastic_constraints = _internal_constraints.template get<ElasticConstraintType>();

    // stiffness
    Vec6r stiffness(_G*_area, _G*_area, _E*_area, _E*_Ix, _E*_Ix, _G*_Iz);

    // create (# Gauss points) constraints per element
    std::array<Real, NUM_GP> gauss_points = GaussQuadratureHelper<NUM_GP>::points();
    std::array<Real, NUM_GP> gauss_weights = GaussQuadratureHelper<NUM_GP>::weights();
    for (int i = 0; i < _num_elements; i++)
    {
        for (unsigned gi = 0; gi < NUM_GP; gi++)
        {
            // get compliance for this constraint
            // stiffness scales according to Gauss quadrature weight
            Vec6r scaled_stiffness = _element_rest_length * gauss_weights[gi] * stiffness;
            Vec6r compliance = 1.0/scaled_stiffness.array();

            elastic_constraints.emplace_back(&_elements[i], gauss_points[gi], compliance);
            // _elastic_constraints.emplace_back(&_nodes[i], &_nodes[i+1], compliance);
        }
    }

    /** Create fixed constraints */
    if (_base_fixed)
    {
        _internal_constraints.template emplace_back<Constraint::OneSidedFixedJointConstraint>(
            _nodes[0].position, _nodes[0].orientation, &_nodes[0], Vec3r::Zero(), Mat3r::Identity()
        );
    }

    if(_tip_fixed)
    {
        _internal_constraints.template emplace_back<Constraint::OneSidedFixedJointConstraint>(
            _nodes.back().position, _nodes.back().orientation, &_nodes.back(), Vec3r::Zero(), Mat3r::Identity()
        );
    }

    /** Add constraints to ordered constraints vector */
    for (unsigned i = 0; i < elastic_constraints.size(); i++)
    {
        _ordered_constraints.emplace_back(&elastic_constraints[i]);
    }

    if (_base_fixed)
    {
        _ordered_constraints.emplace(_ordered_constraints.begin(),
            &_internal_constraints.template get<Constraint::OneSidedFixedJointConstraint>().front());
    }
    if (_tip_fixed)
    {
        _ordered_constraints.emplace(_ordered_constraints.end(),
            &_internal_constraints.template get<Constraint::OneSidedFixedJointConstraint>().back());
    }

    _num_constraints = elastic_constraints.size() + (int)_base_fixed + (int)_tip_fixed;

    /** Allocate space */
    _RHS_vec = VecXr::Zero(6*_num_constraints);
    _alpha.conservativeResize(6*_num_constraints);
    _internal_lambda = VecXr::Zero(6*_num_constraints);
    _dlam = VecXr::Zero(6*_num_constraints);
    _dx.conservativeResize(12*_num_nodes);
    _delC_mat = MatXr::Zero(6*_num_constraints, 12*_num_nodes);

    /** Assemble global inertia vector */
    _inertia_mat_inv = VecXr::Zero(12*_num_nodes);
    for (int i = 0; i < _num_nodes; i++)
    {
        _inertia_mat_inv.block<3,1>(12*i, 0) = Vec3r::Ones() / _nodes[i].mass;
        _inertia_mat_inv.block<3,1>(12*i+3, 0) = Vec3r(1/_nodes[i].Ib[0], 1/_nodes[i].Ib[1], 1/_nodes[i].Ib[2]);
        _inertia_mat_inv.block<3,1>(12*i+6, 0) = 1/_dp_DOF[i].mass.array();
        _inertia_mat_inv.block<3,1>(12*i+9, 0) = 1/_dR_DOF[i].mass.array();
    }

    /** Ensure proper setup of block banded solver */
    int bandwidth = 2*NUM_GP - 1;
    _gradient_buffer.reserve(elastic_constraints.size());

    // number of diagonals = bandwidth + 1
    _diagonals.resize(bandwidth+1);
    for (int i = 0; i < bandwidth+1; i++)
        _diagonals[i].resize(_num_constraints, Mat6r::Zero());

    _solver.setBandwidth(bandwidth);
    _solver.setNumDiagBlocks(_num_constraints);
}

void XPBDCubicHermiteRod::inertialUpdate(Real dt)
{
    std::cout << "\n\n" << std::endl;
    for (int i = 0; i < _num_nodes; i++)
    {
        std::cout << "dp " << i << " before inertial update: " << _dp_DOF[i].position.transpose() << std::endl;
        std::cout << "dR " << i << " before inertial update: " << _dR_DOF[i].position.transpose() << std::endl;

        auto& node = _nodes[i];

        Vec3r F_ext = node.mass * Vec3r(0,-G_ACCEL,0);
        Vec3r T_ext = Vec3r(0,0,0);
        node.inertialUpdate(dt, F_ext, T_ext);


        /** FOR NOW, NO APPLIED FORCE TO DERIVATIVE DOF
         * 
         * Need to look at virtual work derivations to see if this is correct.
         */
        if (i == _num_nodes-1 && !_tip_fixed)
        {
            Vec3r F_ext = node.mass * Vec3r(0, -G_ACCEL/12.0, 0);
            _dp_DOF[i].inertialUpdate(dt, F_ext);
        }
        else
        {
            _dp_DOF[i].inertialUpdate(dt, Vec3r::Zero());
        }
        
        _dR_DOF[i].inertialUpdate(dt, Vec3r::Zero());

        std::cout << "dp " << i << " after inertial update: " << _dp_DOF[i].position.transpose() << std::endl;
        std::cout << "dR " << i << " after inertial update: " << _dR_DOF[i].position.transpose() << std::endl;
    }

    
}

void XPBDCubicHermiteRod::velocityUpdate(Real dt)
{
    for (unsigned i = 0; i < _nodes.size(); i++)
    {
        _nodes[i].velocityUpdate(dt);
        _dp_DOF[i].velocityUpdate(dt);
        _dR_DOF[i].velocityUpdate(dt);
    }

    _internal_lambda = VecXr::Zero(6*_num_constraints);
}

void XPBDCubicHermiteRod::internalConstraintSolve(Real dt)
{
    // if we are not solving the system globally (i.e. using Gauss-Seidel or other iterative method instead),
    // don't do the internal constraint solve
    // assume that we have added the constraints to the top-level Gauss-Seidel solver, and let it do the work
    if (!_global_solve)
        return;

    // get a reference to the elastic constraints
    std::vector<ElasticConstraintType>& elastic_constraints = _internal_constraints.template get<ElasticConstraintType>();

    /** Assemble diagonal blocks for solver */

    // // iterate through elements and compute all the constraint gradients
    // for (int i = 0; i < _num_elements; i++)
    // {
    //     for (int j = 0; j < NUM_GP; j++)
    //     {
    //         int constraint_ind = NUM_GP*i + j;
    //         _gradient_buffer[constraint_ind] = elastic_constraints[constraint_ind].gradient();
    //     }
    // }

    // // iterate through again and assemble the diagonals of the system matrix and the RHS vector
    // int diag_block_ind = 0;

    // // if the base is fixed, compute the entries in the system matrix
    // // the fixed base constraint gradient overlaps with just the constraint gradients of the first element, 
    // //    and only w.r.t. the first node
    // if (_base_fixed)
    // {
    //     // evaluate the fixed base constraint
    //     const Constraint::OneSidedFixedJointConstraint& fixed_base_constraint = 
    //         _internal_constraints.get<Constraint::OneSidedFixedJointConstraint>().front();

    //     // compute the RHS associated with the constraint
    //     typename Constraint::OneSidedFixedJointConstraint::ConstraintVecType fixed_C = fixed_base_constraint.evaluate();
    //     typename Constraint::OneSidedFixedJointConstraint::AlphaVecType fixed_alpha_tilde = fixed_base_constraint.alpha() / (dt * dt);
    //     _RHS_vec.block<6,1>(6*diag_block_ind, 0) = -fixed_C - fixed_alpha_tilde.asDiagonal() * _internal_lambda.block<6,1>(6*diag_block_ind, 0);

    //     // compute the gradient of the fixed base constraint
    //     typename Constraint::OneSidedFixedJointConstraint::GradientMatType fixed_grad = 
    //         _internal_constraints.get<Constraint::OneSidedFixedJointConstraint>().front().gradient();
        
    //     // main diagonal is just delC * M^-1 * delC^T + alpha_tilde
    //     _diagonals[0][diag_block_ind] = fixed_grad * _node_inverse_inertias.front().asDiagonal() * fixed_grad.transpose();
    //     _diagonals[0][diag_block_ind].diagonal() += fixed_alpha_tilde;

    //     // off diagonals are just the blocks of the gradients corresponding to the first node in the
    //     //  delC_element * M1^-1 * delC_fixed^T
    //     for (int j = 0; j < NUM_GP; j++)
    //     {
    //         Mat6r node1_block = _gradient_buffer[j].template block<6,6>(0,0);
    //         _diagonals[j+1][diag_block_ind] = node1_block * _node_inverse_inertias.front().asDiagonal() * fixed_grad.transpose();
    //     }

    //     diag_block_ind++;
    // }

    // for (int i = 0; i < _num_elements; i++)
    // {
    //     // assemble element inertia
    //     Eigen::Vector<Real, 6*(NUM_EN)> element_inverse_inertia;
    //     for (int k = 0; k < NUM_EN; k++)
    //     {
    //         int node_ind = i*(NUM_EN-1) + k;
    //         element_inverse_inertia.template block<6,1>(6*k,0) = _node_inverse_inertias[node_ind];
    //     }

    //     for (int j = 0; j < NUM_GP; j++)
    //     {
    //         // index of the current elastic constraint we are on
    //         int this_ind = NUM_GP*i + j;

    //         // compute RHS
    //         typename ElasticConstraintType::AlphaVecType alpha_tilde = elastic_constraints[this_ind].alpha() / (dt*dt);
    //         _RHS_vec.template block<6,1>(6*diag_block_ind, 0) = -elastic_constraints[this_ind].evaluate() - alpha_tilde.asDiagonal() * _internal_lambda.template block<6,1>(6*diag_block_ind, 0);

    //         // last constraint index that is still in this same element (element i)
    //         int end_of_this_element_ind = std::min(this_ind + (NUM_GP - j - 1), _num_elements * NUM_GP);
    //         // last constraint index that is in the next element (element i+1)
    //         int end_of_next_element_ind = std::min(this_ind + (NUM_GP - j - 1) + NUM_GP, _num_elements * NUM_GP);

    //         // for constraints defined on the same element, the gradients overlap for all nodes
    //         // therefore we can simply take the delC^T * delC product of the entire gradient matrix
    //         for (int other = this_ind; other <= end_of_this_element_ind; other++)
    //         {
    //             int diag_index = other - this_ind;
    //             _diagonals[diag_index][diag_block_ind] = 
    //                 _gradient_buffer[other] * element_inverse_inertia.asDiagonal() * _gradient_buffer[this_ind].transpose();
    //         }

    //         // for constraints defined on adjacent elements, the gradients will only overlap for a single node
    //         // this will be the "last" node of the constraint we are currently on, and the "first" node of the other constraint we are multiplying with
    //         for (int other = end_of_this_element_ind+1; other <= end_of_next_element_ind; other++)
    //         {
    //             int diag_index = other - this_ind;

    //             // the index of the node shared by the constraints
    //             int shared_node_ind = (i+1)*(NUM_EN-1);

    //             // extract the block associated with the "last" node affected by the constraint
    //             Mat6r this_block = _gradient_buffer[this_ind].template block<6,6>(0,6*(NUM_EN-1));
    //             // extract the block associated with the "first" node affected by the constraint
    //             Mat6r other_block = _gradient_buffer[other].template block<6,6>(0,0);

    //             _diagonals[diag_index][diag_block_ind] = other_block * _node_inverse_inertias[shared_node_ind].asDiagonal() * this_block.transpose();
    //         }

    //         // add alpha to main diagonal
    //         _diagonals[0][diag_block_ind].diagonal() += alpha_tilde;

    //         diag_block_ind++;
    //     }
    // }

    // // if the tip is fixed, compute the entries in the system matrix
    // // the fixed tip constraint gradient overlaps with just the constraint gradients of the last element, 
    // //    and only w.r.t. the last node
    // if (_tip_fixed)
    // {
    //     // evaluate the fixed base constraint
    //     const Constraint::OneSidedFixedJointConstraint& fixed_tip_constraint = 
    //         _internal_constraints.get<Constraint::OneSidedFixedJointConstraint>().back();

    //     // compute the RHS associated with the constraint
    //     typename Constraint::OneSidedFixedJointConstraint::ConstraintVecType fixed_C = fixed_tip_constraint.evaluate();
    //     typename Constraint::OneSidedFixedJointConstraint::AlphaVecType fixed_alpha_tilde = fixed_tip_constraint.alpha() / (dt * dt);
    //     _RHS_vec.template block<6,1>(6*diag_block_ind, 0) = -fixed_C - fixed_alpha_tilde.asDiagonal() * _internal_lambda.template block<6,1>(6*diag_block_ind, 0);

    //     // compute the gradient of the fixed base constraint
    //     typename Constraint::OneSidedFixedJointConstraint::GradientMatType fixed_grad = fixed_tip_constraint.gradient();
        
    //     // main diagonal is just delC * M^-1 * delC^T + alpha_tilde
    //     _diagonals[0][diag_block_ind] = fixed_grad * _node_inverse_inertias.back().asDiagonal() * fixed_grad.transpose();
    //     _diagonals[0][diag_block_ind].diagonal() += fixed_alpha_tilde;

    //     // off diagonals are just the blocks of the gradients corresponding to the last node in the
    //     //  delC_element * M1^-1 * delC_fixed^T
    //     for (int j = 0; j < NUM_GP; j++)
    //     {
    //         Mat6r node1_block = _gradient_buffer[_num_elements-NUM_GP+j].template block<6,6>(0,6*(NUM_EN-1));
    //         _diagonals[j+1][diag_block_ind-(j+1)] = fixed_grad * _node_inverse_inertias.back().asDiagonal() * node1_block.transpose();
    //     }

    //     diag_block_ind++;
    // }

    // // solve system
    // _solver.solveInPlace(_diagonals, _RHS_vec, _dlam);
    // // std::cout << "\ndlam banded solver: " << _dlam.transpose() << std::endl;

    // // compute position updates
    // _dx = VecXr::Zero(6*_num_nodes);
    // int constraint_ind = 0;
    // if (_base_fixed)
    // {
    //     // evaluate the fixed base constraint
    //     const Constraint::OneSidedFixedJointConstraint& fixed_base_constraint = 
    //         _internal_constraints.get<Constraint::OneSidedFixedJointConstraint>().front();

    //     Vec6r constraint_dlam = _dlam.template block<6,1>(6*constraint_ind, 0);
    //     _dx.template block<6,1>(0,0) += _node_inverse_inertias.front().asDiagonal() * fixed_base_constraint.gradient().transpose() * constraint_dlam;
        
    //     constraint_ind++;
    // }


    // for (int i = 0; i < _num_elements; i++)
    // {
    //     int first_node_ind = i*(NUM_EN-1);
    //     for (int j = 0; j < NUM_GP; j++)
    //     {
    //         // index of the current elastic constraint we are on
    //         int elastic_constraint_ind = NUM_GP*i + j;

    //         // iterate through the nodes of the elastic constraint and compute position updates
    //         for (int k = 0; k < NUM_EN; k++)
    //         {
    //             int node_ind = first_node_ind + k;
    //             Mat6r delC_block = _gradient_buffer[elastic_constraint_ind].template block<6,6>(0,6*k);
    //             Vec6r constraint_dlam = _dlam.template block<6,1>(6*constraint_ind, 0);

    //             _dx.template block<6,1>(6*node_ind,0) += 
    //                 _node_inverse_inertias[node_ind].asDiagonal() * delC_block.transpose() * constraint_dlam;
    //         }

    //         constraint_ind++;
    //     }
    // }

    // if (_tip_fixed)
    // {
    //     // evaluate the fixed base constraint
    //     const Constraint::OneSidedFixedJointConstraint& fixed_tip_constraint = 
    //         _internal_constraints.get<Constraint::OneSidedFixedJointConstraint>().back();

    //     Vec6r constraint_dlam = _dlam.template block<6,1>(6*constraint_ind, 0);
    //     // std::cout << "constraint dlam: " << constraint_dlam << std::endl;
    //     _dx.template block<6,1>(6*(_num_nodes-1),0) += _node_inverse_inertias.back().asDiagonal() * fixed_tip_constraint.gradient().transpose() * constraint_dlam;

    //     // std::cout << "dx tip fixed: " << _node_inverse_inertias.back().asDiagonal() * fixed_tip_constraint.gradient().transpose() * constraint_dlam << std::endl;
    // }

    // std::cout << "dx banded solver: " << _dx.transpose() << std::endl;


    int constraint_index = 0;
    for (const auto& constraint_variant : _ordered_constraints)
    {
        std::visit([&](const auto& constraint) {
            using ConstraintType = base_type_t<decltype(constraint)>;

            // evaluate the constraint and put it in global constraint vector
            typename ConstraintType::ConstraintVecType vec = constraint->evaluate();
            _RHS_vec.template block<ConstraintType::ConstraintDim,1>(constraint_index,0) = -vec;

            _alpha.template block<ConstraintType::ConstraintDim,1>(constraint_index,0) = constraint->alpha();

            // evaluate the gradient and put it in global delC matrix
            typename ConstraintType::GradientMatType gradient = constraint->gradient();
            for (int i = 0; i < ConstraintType::NumOrientedParticles; i++)
            {
                int particle_index = constraint->orientedParticles()[i] - _nodes.data();
                _delC_mat.template block<ConstraintType::ConstraintDim, 6>(constraint_index, 12*particle_index) = 
                    gradient.template block<ConstraintType::ConstraintDim, 6>(0, 6*i);
            }
            for (int i = 0; i < ConstraintType::NumParticles; i++)
            {
                std::cout << "constraint->particles()[i]: " << constraint->particles()[i] << std::endl;
                std::cout << "_dp_DOF.data(): " << _dp_DOF.data() << std::endl;
                std::cout << "_dR_DOF.data(): " << _dR_DOF.data() << std::endl;
                int particle_index_dp = constraint->particles()[i] - _dp_DOF.data();
                int particle_index_dR = constraint->particles()[i] - _dR_DOF.data();
                int particle_index;
                int DOF_index;
                if (particle_index_dp >= 0 && particle_index_dp < _num_nodes)
                {
                    particle_index = particle_index_dp;
                    DOF_index = 12*particle_index + 6;
                }
                else
                {
                    particle_index = particle_index_dR;
                    DOF_index = 12*particle_index + 9;
                }
                
                std::cout << "particle index: " << particle_index << std::endl;
                std::cout << "global DOF index start: " << DOF_index << std::endl;

                _delC_mat.template block<ConstraintType::ConstraintDim, 3>(constraint_index, DOF_index) = 
                    gradient.template block<ConstraintType::ConstraintDim, 3>(0, 6*ConstraintType::NumOrientedParticles+3*i);
            }

            constraint_index += ConstraintType::ConstraintDim;
        },
        constraint_variant);
    }

    // std::cout << "\nRodElement 1 strain: " << (_elements[1].strain(0.5) - Vec6r(0,0,1,0,0,0)).transpose() << std::endl;
    // std::cout << "Elastic constraint 1 strain: " << _elastic_constraints[1].evaluate().transpose() << std::endl;
    // std::cout << "RodElement 1 gradient:\n" << _elements[1].strainGradient(0.5) << std::endl;
    // std::cout << "Elastic constraint 1 gradient:\n" << _elastic_constraints[1].gradient() << std::endl;

    // Step 4: assemble and solve
    // compute LHS
    VecXr alpha_tilde = _alpha/(dt*dt);
    // std::cout << "Alpha tilde: " << alpha_tilde.transpose() << std::endl;
    // std::cout << "RHS: " << _RHS_vec.transpose() << std::endl;
    // std::cout << "inertia mat inv: " << _inertia_mat_inv.transpose() << std::endl;
    // std::cout << "DelC mat:\n" << _delC_mat << std::endl;
    MatXr LHS_mat = _delC_mat * _inertia_mat_inv.asDiagonal() * _delC_mat.transpose();
    LHS_mat.diagonal() += alpha_tilde;
    // std::cout << "LHS mat:\n" << LHS_mat << std::endl;

    _RHS_vec -= alpha_tilde.asDiagonal() * _internal_lambda;

    Eigen::LLT<MatXr> llt(LHS_mat);
    // Eigen::SelfAdjointEigenSolver<MatXr> eig(LHS_mat);
    // std::cout << "Eigenvalues: " << eig.eigenvalues().transpose() << std::endl;
    // if (eig.eigenvalues().minCoeff() <= 0) {
    //     std::cerr << "Matrix is not positive definite!" << std::endl;
    // }
    VecXr dlam = llt.solve(_RHS_vec);
    _dx = _inertia_mat_inv.asDiagonal() * _delC_mat.transpose() * dlam;
    // std::cout << "dlam global: " << dlam.transpose() << std::endl;
    std::cout << "dx global: " << _dx.transpose() << std::endl;

    _internal_lambda += _dlam;

    for (int i = 0; i < _num_nodes; i++)
    {
        Vec3r dp = _dx( Eigen::seqN(12*i,3) );
        Vec3r dor = _dx( Eigen::seqN(12*i+3,3) );
        _nodes[i].positionUpdate(dp, dor);

        Vec3r ddp_ds = _dx(Eigen::seqN(12*i+6,3));
        Vec3r ddR_ds = _dx(Eigen::seqN(12*i+9,3));
        _dp_DOF[i].positionUpdate(ddp_ds);
        _dR_DOF[i].positionUpdate(ddR_ds); 
    }
}

} // namespace SimObject