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
    _length(config.length()), _radius(config.diameter()/2.0),
    _base_fixed(config.baseFixed()), _tip_fixed(config.tipFixed()),
    _density(config.density()), _E(config.E()), _nu(config.nu()),
    _solver(Order, config.nodes())
{
    // compute shear modulus
    _G = _E / (2 * (1+_nu));

    // compute cross section properties
    _area = M_PI * _radius * _radius;
    _Ix = M_PI * _radius * _radius * _radius * _radius / 4.0;
    _Iz = 2*_Ix;

    // create elements for the rod
    /** TODO: For now, the nodes() parameter in the config file will be used to specify the number of elements */
    _num_elements = config.nodes()-1;
    // total number of nodes = internal nodes ((Order - 1) of these per element) + element boundary nodes (number of elements + 1 of these)
    _num_nodes = _num_elements * (Order - 1) + (_num_elements+1);

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

    for (int i = 0; i < _num_nodes; i++)
    {
        std::cout << "node " << i << " inertia:\n" << _nodes[i].mass << ",\n" << _nodes[i].Ib.transpose() << std::endl;
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
    std::array<Real, Order> gauss_points = GaussQuadratureHelper<Order>::points();
    std::array<Real, Order> gauss_weights = GaussQuadratureHelper<Order>::weights();
    for (int i = 0; i < _num_elements; i++)
    {
        for (unsigned gi = 0; gi < gauss_points.size(); gi++)
        {
            // get compliance for this constraint
            // stiffness scales according to Gauss quadrature weight
            Vec6r scaled_stiffness = _element_rest_length * gauss_weights[gi] * stiffness;
            Vec6r compliance = 1.0/scaled_stiffness.array();

            std::cout << "Constraint " << i << " compliance: " << compliance.transpose() << std::endl;

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

    _num_constraints = _elastic_constraints.size() + 1;

    /** Allocate space */
    _RHS_vec.conservativeResize(6*_num_constraints);
    _alpha.conservativeResize(6*_num_constraints);
    _internal_lambda = VecXr::Zero(6*_num_constraints);
    _dlam.conservativeResize(6*_num_constraints);
    _dx.conservativeResize(6*_num_nodes);
    _delC_mat.conservativeResize(6*_num_constraints, 6*_num_nodes);

    /** Assemble global inertia vector */
    _inertia_mat_inv = VecXr::Zero(6*_num_nodes);
    for (int i = 0; i < _num_nodes; i++)
    {
        _inertia_mat_inv.block<3,1>(i*6, 0) = Vec3r::Ones() / _nodes[i].mass;
        _inertia_mat_inv.block<3,1>(i*6+3, 0) = Vec3r(1/_nodes[i].Ib[0], 1/_nodes[i].Ib[1], 1/_nodes[i].Ib[2]);
    }
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
            for (int i = 0; i < ConstraintType::NumParticles; i++)
            {
                int particle_index = constraint->particles()[i] - _nodes.data();
                _delC_mat.template block<ConstraintType::ConstraintDim, 6>(constraint_index, 6*particle_index) = 
                    gradient.template block<ConstraintType::ConstraintDim, 6>(0, 6*i);
            }

            constraint_index += ConstraintType::ConstraintDim;
        },
        constraint_variant);
    }

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
    // std::cout << "Dlam: " << dlam.transpose() << std::endl;
    // std::cout << "dx: " << _dx.transpose() << std::endl;

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