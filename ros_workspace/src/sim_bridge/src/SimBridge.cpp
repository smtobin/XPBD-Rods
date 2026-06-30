#include "sim_bridge/SimBridge.hpp"

SimBridge::SimBridge(Sim::Simulation* sim)
    : rclcpp::Node("sim_bridge"), _sim(sim)
{

}