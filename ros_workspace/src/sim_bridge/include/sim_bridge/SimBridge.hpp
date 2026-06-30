#pragma once

#include "simulation/Simulation.hpp"

#include <rclcpp/rclcpp.hpp>

class SimBridge : public rclcpp::Node
{

public:
    SimBridge(Sim::Simulation* sim);

protected:
    Sim::Simulation* _sim;

};