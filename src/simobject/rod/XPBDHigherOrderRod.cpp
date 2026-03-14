#include "simobject/rod/XPBDHigherOrderRod.hpp"

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

template <int Order>
XPBDRod_<Order>::XPBDRod_(const Config::RodConfig& config)
    : XPBDObject_Base(config),
    _num_elements(config.elements()),
    _num_nodes(_num_elements * (Order - 1) + (_num_elements+1)),
    _length(config.length()), _radius(config.diameter()/2.0),
    _base_fixed(config.baseFixed()), _tip_fixed(config.tipFixed()),
    _density(config.density()), _E(config.E()), _nu(config.nu()),
    _solver((3*NUM_GP)/2, _num_nodes)
{
    // compute shear modulus
    _G = _E / (2 * (1+_nu));

    // compute cross section properties
    _area = M_PI * _radius * _radius;
    _Ix = M_PI * _radius * _radius * _radius * _radius / 4.0;
    _Iz = 2*_Ix;

    // element rest length
    _element_rest_length = _length/(_num_nodes-1) * Order;

    /** Create nodes */
    _nodes.resize(_num_nodes);

    // create base node
    _nodes[0].position = config.initialPosition();
    _nodes[0].lin_velocity = config.initialVelocity();
    _nodes[0].orientation = Math::RotMatFromXYZEulerAngles(config.initialRotation());
    _nodes[0].ang_velocity = config.initialAngularVelocity();
    _nodes[0].mass = 0;
    _nodes[0].Ib = Vec3r::Zero();
    _nodes[0].prev_position = _nodes[0].position;
    _nodes[0].prev_orientation = _nodes[0].orientation;

    // create the rest of the nodes
    // leave inertial properties empty - will fill in later
    for (int i = 1; i < _num_nodes; i++)
    {
        _nodes[i].position = _nodes[i-1].position + _nodes[i-1].orientation * Vec3r(0,0,_length/(_num_nodes-1));
        _nodes[i].lin_velocity = _nodes[i-1].lin_velocity;
        _nodes[i].orientation = _nodes[i-1].orientation;
        _nodes[i].ang_velocity = _nodes[i-1].ang_velocity;
        _nodes[i].mass = 0;
        _nodes[i].Ib = Vec3r::Zero();
        _nodes[i].prev_position = _nodes[i].position;
        _nodes[i].prev_orientation = _nodes[i].orientation;
    }


    /** Create elements */
    _elements.reserve(_num_elements);

    // lumped mass proportions
    std::array<Real, Order+1> lumped_masses = RodElement<Order>::lumpedMasses();

    for (int i = 0; i < _num_elements; i++)
    {
        // total element mass
        Real total_mass = _element_rest_length * _area * _density;
        Vec3r total_rot_inertia = _density * _element_rest_length * Vec3r(_Ix, _Ix, _Iz);

        

        // this element is composed of nodes i*Order through (i+1)*Order
        std::array<OrientedParticle*, Order+1> element_nodes;
        for (int j = 0; j <= Order; j++)
        {
            int node_ind = i*Order + j;
            element_nodes[j] = &_nodes[node_ind];

            _nodes[node_ind].mass += total_mass * lumped_masses[j];
            _nodes[node_ind].Ib += total_rot_inertia * lumped_masses[j];
        }

        _elements.emplace_back(element_nodes, _element_rest_length);
    }

    // compute inverse inertias
    _node_inverse_inertias.resize(_num_nodes);
    for (int i = 0; i < _num_nodes; i++)
    {
        _node_inverse_inertias[i] = Vec6r(1.0/_nodes[i].mass, 1.0/_nodes[i].mass, 1.0/_nodes[i].mass, 1.0/_nodes[i].Ib[0], 1.0/_nodes[i].Ib[1], 1.0/_nodes[i].Ib[2]);
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

}

template <int Order>
std::vector<const OrientedParticle*> XPBDRod_<Order>::particles() const
{
    std::vector<const OrientedParticle*> particles_vec;
    particles_vec.reserve(_nodes.size());
    for (const auto& node : _nodes)
    {
        particles_vec.push_back(&node);
    }

    return particles_vec;
}

template <int Order>
AABB XPBDRod_<Order>::boundingBox() const
{
    AABB bbox;
    bbox.min = _nodes.front().position;
    bbox.max = _nodes.front().position;

    for (int i = 1; i < _num_nodes; i++)
    {
        bbox.min[0] = std::min(bbox.min[0], _nodes[i].position[0]);
        bbox.min[1] = std::min(bbox.min[1], _nodes[i].position[1]);
        bbox.min[2] = std::min(bbox.min[2], _nodes[i].position[2]);

        bbox.max[0] = std::max(bbox.max[0], _nodes[i].position[0]);
        bbox.max[1] = std::max(bbox.max[1], _nodes[i].position[1]);
        bbox.max[2] = std::max(bbox.max[2], _nodes[i].position[2]);
    }

    return bbox;
}

template <int Order>
void XPBDRod_<Order>::setup()
{
    /** Create elastic constraints */

    // stiffness
    Vec6r stiffness(_G*_area, _G*_area, _E*_area, _E*_Ix, _E*_Ix, _G*_Iz);

    // create (# Gauss points) constraints per element
    std::array<Real, NUM_GP> gauss_points = GaussQuadratureHelper<Order>::points();
    std::array<Real, NUM_GP> gauss_weights = GaussQuadratureHelper<Order>::weights();
    for (int i = 0; i < _num_elements; i++)
    {
        for (unsigned gi = 0; gi < gauss_points.size(); gi++)
        {
            // get compliance for this constraint
            // stiffness scales according to Gauss quadrature weight
            Vec6r scaled_stiffness = _element_rest_length * gauss_weights[gi] * stiffness;
            Vec6r compliance = 1.0/scaled_stiffness.array();

            _elastic_constraints.emplace_back(&_elements[i], gauss_points[gi], compliance);
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
    for (unsigned i = 0; i < _elastic_constraints.size(); i++)
    {
        _ordered_constraints.emplace_back(&_elastic_constraints[i]);
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

    _num_constraints = _elastic_constraints.size() + (int)_base_fixed + (int)_tip_fixed;

    /** Allocate space */
    _RHS_vec = VecXr::Zero(6*_num_constraints);
    _alpha.conservativeResize(6*_num_constraints);
    _internal_lambda = VecXr::Zero(6*_num_constraints);
    _dlam = VecXr::Zero(6*_num_constraints);
    _dx.conservativeResize(6*_num_nodes);
    _delC_mat = MatXr::Zero(6*_num_constraints, 6*_num_nodes);

    /** Assemble global inertia vector */
    _inertia_mat_inv = VecXr::Zero(6*_num_nodes);
    for (int i = 0; i < _num_nodes; i++)
    {
        _inertia_mat_inv.block<3,1>(i*6, 0) = Vec3r::Ones() / _nodes[i].mass;
        _inertia_mat_inv.block<3,1>(i*6+3, 0) = Vec3r(1/_nodes[i].Ib[0], 1/_nodes[i].Ib[1], 1/_nodes[i].Ib[2]);
    }

    /** Ensure proper setup of block banded solver */
    int bandwidth = (3*NUM_GP)/2;
    _gradient_buffer.reserve(_elastic_constraints.size());

    // number of diagonals = bandwidth + 1
    _diagonals.resize(bandwidth+1);
    for (int i = 0; i < bandwidth+1; i++)
        _diagonals[i].resize(_num_constraints);

    _solver.setBandwidth(bandwidth);
    _solver.setNumDiagBlocks(_num_constraints);
}

template <int Order>
void XPBDRod_<Order>::inertialUpdate(Real dt)
{
    for (int i = 0; i < _num_nodes; i++)
    {
        auto& node = _nodes[i];

        Vec3r F_ext = node.mass * Vec3r(0,-G_ACCEL,0);
        Vec3r T_ext = Vec3r(0,0,0);
        node.inertialUpdate(dt, F_ext, T_ext);
    }
}

template <int Order>
void XPBDRod_<Order>::velocityUpdate(Real dt)
{
    for (unsigned i = 0; i < _nodes.size(); i++)
    {
        _nodes[i].velocityUpdate(dt);
    }

    _internal_lambda = VecXr::Zero(6*_num_constraints);
}

template <int Order>
void XPBDRod_<Order>::internalConstraintSolve(Real dt)
{
    /** Assemble diagonal blocks for solver */

    // iterate through elements and compute all the constraint gradients
    for (int i = 0; i < _num_elements; i++)
    {
        for (int j = 0; j < NUM_GP; j++)
        {
            int constraint_ind = NUM_GP*i + j;
            _gradient_buffer[constraint_ind] = _elastic_constraints[constraint_ind].gradient();
        }
    }

    // iterate through again and assemble the diagonals of the system matrix and the RHS vector
    int diag_block_ind = 0;

    // if the base is fixed, compute the entries in the system matrix
    // the fixed base constraint gradient overlaps with just the constraint gradients of the first element, 
    //    and only w.r.t. the first node
    if (_base_fixed)
    {
        // evaluate the fixed base constraint
        const Constraint::OneSidedFixedJointConstraint& fixed_base_constraint = 
            _internal_constraints.get<Constraint::OneSidedFixedJointConstraint>().front();

        // compute the RHS associated with the constraint
        typename Constraint::OneSidedFixedJointConstraint::ConstraintVecType fixed_C = fixed_base_constraint.evaluate();
        typename Constraint::OneSidedFixedJointConstraint::AlphaVecType fixed_alpha_tilde = fixed_base_constraint.alpha() / (dt * dt);
        _RHS_vec.block<6,1>(6*diag_block_ind, 0) = -fixed_C - fixed_alpha_tilde.asDiagonal() * _internal_lambda.block<6,1>(6*diag_block_ind, 0);

        // compute the gradient of the fixed base constraint
        typename Constraint::OneSidedFixedJointConstraint::GradientMatType fixed_grad = 
            _internal_constraints.get<Constraint::OneSidedFixedJointConstraint>().front().gradient();
        
        // main diagonal is just delC * M^-1 * delC^T + alpha_tilde
        _diagonals[0][diag_block_ind] = fixed_grad * _node_inverse_inertias.front().asDiagonal() * fixed_grad.transpose();
        _diagonals[0][diag_block_ind].diagonal() += fixed_alpha_tilde;

        // off diagonals are just the blocks of the gradients corresponding to the first node in the
        //  delC_element * M1^-1 * delC_fixed^T
        for (int j = 0; j < NUM_GP; j++)
        {
            Mat6r node1_block = _gradient_buffer[j].template block<6,6>(0,0);
            _diagonals[j+1][diag_block_ind] = node1_block * _node_inverse_inertias.front().asDiagonal() * fixed_grad.transpose();
        }

        diag_block_ind++;
    }

    for (int i = 0; i < _num_elements; i++)
    {
        // assemble element inertia
        Eigen::Vector<Real, 6*(Order+1)> element_inverse_inertia;
        for (int k = 0; k < Order+1; k++)
        {
            int node_ind = i*Order + k;
            element_inverse_inertia.template block<6,1>(6*k,0) = _node_inverse_inertias[node_ind];
        }

        for (int j = 0; j < NUM_GP; j++)
        {
            // index of the current elastic constraint we are on
            int this_ind = NUM_GP*i + j;

            // compute RHS
            typename Constraint::RodElasticGaussPointConstraint<Order>::AlphaVecType alpha_tilde = _elastic_constraints[this_ind].alpha() / (dt*dt);
            _RHS_vec.template block<6,1>(6*diag_block_ind, 0) = -_elastic_constraints[this_ind].evaluate() - alpha_tilde.asDiagonal() * _internal_lambda.template block<6,1>(6*diag_block_ind, 0);

            // last constraint index that is still in this same element (element i)
            int end_of_this_element_ind = std::min(this_ind + (NUM_GP - j - 1), _num_elements * NUM_GP);
            // last constraint index that is in the next element (element i+1)
            int end_of_next_element_ind = std::min(this_ind + (NUM_GP - j - 1) + NUM_GP, _num_elements * NUM_GP);

            // for constraints defined on the same element, the gradients overlap for all nodes
            // therefore we can simply take the delC^T * delC product of the entire gradient matrix
            for (int other = this_ind; other <= end_of_this_element_ind; other++)
            {
                int diag_index = other - this_ind;
                _diagonals[diag_index][diag_block_ind] = 
                    _gradient_buffer[other] * element_inverse_inertia.asDiagonal() * _gradient_buffer[this_ind].transpose();
            }

            // for constraints defined on adjacent elements, the gradients will only overlap for a single node
            // this will be the "last" node of the constraint we are currently on, and the "first" node of the other constraint we are multiplying with
            for (int other = end_of_this_element_ind+1; other <= end_of_next_element_ind; other++)
            {
                int diag_index = other - this_ind;

                // the index of the node shared by the constraints
                int shared_node_ind = (i+1)*Order;

                // extract the block associated with the "last" node affected by the constraint
                Mat6r this_block = _gradient_buffer[this_ind].template block<6,6>(0,6*(Order));
                // extract the block associated with the "first" node affected by the constraint
                Mat6r other_block = _gradient_buffer[other].template block<6,6>(0,0);

                _diagonals[diag_index][diag_block_ind] = other_block * _node_inverse_inertias[shared_node_ind].asDiagonal() * this_block.transpose();
            }

            // add alpha to main diagonal
            _diagonals[0][diag_block_ind].diagonal() += alpha_tilde;

            diag_block_ind++;
        }
    }

    // if the tip is fixed, compute the entries in the system matrix
    // the fixed tip constraint gradient overlaps with just the constraint gradients of the last element, 
    //    and only w.r.t. the last node
    if (_tip_fixed)
    {
        // evaluate the fixed base constraint
        const Constraint::OneSidedFixedJointConstraint& fixed_tip_constraint = 
            _internal_constraints.get<Constraint::OneSidedFixedJointConstraint>().back();

        // compute the RHS associated with the constraint
        typename Constraint::OneSidedFixedJointConstraint::ConstraintVecType fixed_C = fixed_tip_constraint.evaluate();
        typename Constraint::OneSidedFixedJointConstraint::AlphaVecType fixed_alpha_tilde = fixed_tip_constraint.alpha() / (dt * dt);
        _RHS_vec.template block<6,1>(6*diag_block_ind, 0) = -fixed_C - fixed_alpha_tilde.asDiagonal() * _internal_lambda.template block<6,1>(6*diag_block_ind, 0);

        // compute the gradient of the fixed base constraint
        typename Constraint::OneSidedFixedJointConstraint::GradientMatType fixed_grad = fixed_tip_constraint.gradient();
        
        // main diagonal is just delC * M^-1 * delC^T + alpha_tilde
        _diagonals[0][diag_block_ind] = fixed_grad * _node_inverse_inertias.back().asDiagonal() * fixed_grad.transpose();
        _diagonals[0][diag_block_ind].diagonal() += fixed_alpha_tilde;

        // off diagonals are just the blocks of the gradients corresponding to the last node in the
        //  delC_element * M1^-1 * delC_fixed^T
        for (int j = 0; j < NUM_GP; j++)
        {
            Mat6r node1_block = _gradient_buffer[_num_elements-NUM_GP+j].template block<6,6>(0,6*(Order));
            _diagonals[j+1][diag_block_ind-(j+1)] = fixed_grad * _node_inverse_inertias.back().asDiagonal() * node1_block.transpose();
        }

        diag_block_ind++;
    }

    // solve system
    _solver.solveInPlace(_diagonals, _RHS_vec, _dlam);
    // std::cout << "\ndlam banded solver: " << _dlam.transpose() << std::endl;

    // compute position updates
    _dx = VecXr::Zero(6*_num_nodes);
    int constraint_ind = 0;
    if (_base_fixed)
    {
        // evaluate the fixed base constraint
        const Constraint::OneSidedFixedJointConstraint& fixed_base_constraint = 
            _internal_constraints.get<Constraint::OneSidedFixedJointConstraint>().front();

        Vec6r constraint_dlam = _dlam.template block<6,1>(6*constraint_ind, 0);
        _dx.template block<6,1>(0,0) += _node_inverse_inertias.front().asDiagonal() * fixed_base_constraint.gradient().transpose() * constraint_dlam;
        
        constraint_ind++;
    }


    for (int i = 0; i < _num_elements; i++)
    {
        int first_node_ind = i*Order;
        for (int j = 0; j < NUM_GP; j++)
        {
            // index of the current elastic constraint we are on
            int elastic_constraint_ind = NUM_GP*i + j;

            // iterate through the nodes of the elastic constraint and compute position updates
            for (int k = 0; k < Order+1; k++)
            {
                int node_ind = first_node_ind + k;
                Mat6r delC_block = _gradient_buffer[elastic_constraint_ind].template block<6,6>(0,6*k);
                Vec6r constraint_dlam = _dlam.template block<6,1>(6*constraint_ind, 0);

                _dx.template block<6,1>(6*node_ind,0) += 
                    _node_inverse_inertias[node_ind].asDiagonal() * delC_block.transpose() * constraint_dlam;
            }

            constraint_ind++;
        }
    }

    if (_tip_fixed)
    {
        // evaluate the fixed base constraint
        const Constraint::OneSidedFixedJointConstraint& fixed_tip_constraint = 
            _internal_constraints.get<Constraint::OneSidedFixedJointConstraint>().back();

        Vec6r constraint_dlam = _dlam.template block<6,1>(6*constraint_ind, 0);
        // std::cout << "constraint dlam: " << constraint_dlam << std::endl;
        _dx.template block<6,1>(6*(_num_nodes-1),0) += _node_inverse_inertias.back().asDiagonal() * fixed_tip_constraint.gradient().transpose() * constraint_dlam;

        // std::cout << "dx tip fixed: " << _node_inverse_inertias.back().asDiagonal() * fixed_tip_constraint.gradient().transpose() * constraint_dlam << std::endl;
    }

    // std::cout << "dx banded solver: " << _dx.transpose() << std::endl;


    // int constraint_index = 0;
    // for (const auto& constraint_variant : _ordered_constraints)
    // {
    //     std::visit([&](const auto& constraint) {
    //         using ConstraintType = base_type_t<decltype(constraint)>;

    //         // evaluate the constraint and put it in global constraint vector
    //         typename ConstraintType::ConstraintVecType vec = constraint->evaluate();
    //         _RHS_vec.template block<ConstraintType::ConstraintDim,1>(constraint_index,0) = -vec;

    //         _alpha.template block<ConstraintType::ConstraintDim,1>(constraint_index,0) = constraint->alpha();

    //         // evaluate the gradient and put it in global delC matrix
    //         typename ConstraintType::GradientMatType gradient = constraint->gradient();
    //         for (int i = 0; i < ConstraintType::NumParticles; i++)
    //         {
    //             int particle_index = constraint->particles()[i] - _nodes.data();
    //             _delC_mat.template block<ConstraintType::ConstraintDim, 6>(constraint_index, 6*particle_index) = 
    //                 gradient.template block<ConstraintType::ConstraintDim, 6>(0, 6*i);
    //         }

    //         constraint_index += ConstraintType::ConstraintDim;
    //     },
    //     constraint_variant);
    // }

    // // std::cout << "\nRodElement 1 strain: " << (_elements[1].strain(0.5) - Vec6r(0,0,1,0,0,0)).transpose() << std::endl;
    // // std::cout << "Elastic constraint 1 strain: " << _elastic_constraints[1].evaluate().transpose() << std::endl;
    // // std::cout << "RodElement 1 gradient:\n" << _elements[1].strainGradient(0.5) << std::endl;
    // // std::cout << "Elastic constraint 1 gradient:\n" << _elastic_constraints[1].gradient() << std::endl;

    // // Step 4: assemble and solve
    // // compute LHS
    // VecXr alpha_tilde = _alpha/(dt*dt);
    // // std::cout << "Alpha tilde: " << alpha_tilde.transpose() << std::endl;
    // // std::cout << "RHS: " << _RHS_vec.transpose() << std::endl;
    // // std::cout << "inertia mat inv: " << _inertia_mat_inv.transpose() << std::endl;
    // // std::cout << "DelC mat:\n" << _delC_mat << std::endl;
    // MatXr LHS_mat = _delC_mat * _inertia_mat_inv.asDiagonal() * _delC_mat.transpose();
    // LHS_mat.diagonal() += alpha_tilde;
    // // std::cout << "LHS mat:\n" << LHS_mat << std::endl;

    // _RHS_vec -= alpha_tilde.asDiagonal() * _internal_lambda;

    // Eigen::LLT<MatXr> llt(LHS_mat);
    // // Eigen::SelfAdjointEigenSolver<MatXr> eig(LHS_mat);
    // // std::cout << "Eigenvalues: " << eig.eigenvalues().transpose() << std::endl;
    // // if (eig.eigenvalues().minCoeff() <= 0) {
    // //     std::cerr << "Matrix is not positive definite!" << std::endl;
    // // }
    // VecXr dlam = llt.solve(_RHS_vec);
    // _dx = _inertia_mat_inv.asDiagonal() * _delC_mat.transpose() * dlam;
    // // std::cout << "dlam global: " << dlam.transpose() << std::endl;
    // std::cout << "dx global: " << _dx.transpose() << std::endl;

    _internal_lambda += _dlam;

    for (int i = 0; i < _num_nodes; i++)
    {
        const Vec3r dp = _dx( Eigen::seqN(6*i,3) );
        const Vec3r dor = _dx( Eigen::seqN(6*i+3,3) );
        _nodes[i].positionUpdate(dp, dor);
    }
}

template class XPBDRod_<1>;
template class XPBDRod_<2>;

} // namespace SimObject