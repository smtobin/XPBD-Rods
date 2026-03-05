#pragma once

#include "config/SimulationConfig.hpp"

namespace Config
{

class RodConvergenceSimulationConfig : public SimulationConfig
{
public:
    explicit RodConvergenceSimulationConfig()
        : SimulationConfig()
    {}

    explicit RodConvergenceSimulationConfig(const YAML::Node& node)
        : SimulationConfig(node)
    {
        _extractParameter("E", node, _rod_E);
        _extractParameter("diameter", node, _rod_dia);
        _extractParameter("length", node, _rod_length);
        _extractParameter("density", node, _rod_density);

        _extractParameter("linear-rod-elements", node, _linear_rod_elements);
        _extractParameter("quadratic-rod-elements", node, _quadratic_rod_elements);
    }

    Real rodE() const { return _rod_E.value; }
    Real rodNu() const { return _rod_nu.value; }
    Real rodDensity() const { return _rod_density.value; }
    Real rodDia() const { return _rod_dia.value; }
    Real rodLength() const { return _rod_length.value; }

    std::vector<int> linearRodElements() const { return _linear_rod_elements.value; }
    std::vector<int> quadraticRodElements() const { return _quadratic_rod_elements.value; }

private:
    ConfigParameter<Real> _rod_E = ConfigParameter<Real>(1e6);
    ConfigParameter<Real> _rod_nu = ConfigParameter<Real>(0.4);
    ConfigParameter<Real> _rod_density = ConfigParameter<Real>(1000);
    ConfigParameter<Real> _rod_dia = ConfigParameter<Real>(0.1);
    ConfigParameter<Real> _rod_length = ConfigParameter<Real>(1.0);

    ConfigParameter<std::vector<int>> _linear_rod_elements = ConfigParameter<std::vector<int>>(std::vector<int>{});
    ConfigParameter<std::vector<int>> _quadratic_rod_elements = ConfigParameter<std::vector<int>>(std::vector<int>{});
};

} // namespace Config