#include "simulation/Simulation.hpp"


int main(int argc, char **argv) 
{
    if (argc > 1)
    {
        std::string config_filename(argv[1]);
        Config::SimulationConfig config(YAML::LoadFile(config_filename));
        Config::SimulationRenderConfig render_config;
        Sim::Simulation sim(config, render_config);
        return sim.run();
    }
    else
    {
        std::cerr << "No config file specified!" << std::endl;
    }
}