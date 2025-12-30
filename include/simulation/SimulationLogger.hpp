#pragma once

#include "common/common.hpp"

#include <string>
#include <fstream>
#include <functional>

namespace Sim
{

/** Responsible for logging information about the simulation as it happens.
 * The Simulation class can add quantities that should be logged (vertex displacement, total energy, etc.) by giving the name of the quantity and
 * a function to evaluate the quantity throughout the simulation.
 * 
 * The output is logged to an output file, with a single space used as the delimiter.
 */
class SimulationLogger
{

public:
    /** A simple struct representing info for a variable to be logged. */
    struct LoggedVariable
    {
        std::string name;   // the name of the variable (will appear in the header)
        std::function<Real()> func; // the function to evaluate the variable (used to populate the value at every time step)

        LoggedVariable(const std::string& name_, std::function<Real()> func_)
            : name(name_), func(func_)
        {}
    };

public:
    /** Constructor simply takes the output filename. */
    explicit SimulationLogger(const std::string& output_filename, Real logging_interval);

    /** Adds a variable to be logged as output.
     * This will create a wrapper lambda around the pointer and call the other overload of addOutput.
     * @param var_name - the name of the variable
     * @param var_ptr - a pointer to the value (i.e. like a pointer to a vertex value or something)
     */
    void addOutput(const std::string& var_name, const Real* var_ptr);
    
    /** Adds a variable to be logged as output.
     * @param var_name - the name of the variable
     * @param func - a lambda for evaluating the variable. This allows computed quantities (e.g. total strain energy) to be logged.
     */
    void addOutput(const std::string& var_name, std::function<Real()> func);

    /** Starts logging the output variables. Writes the header to file. */
    void startLogging();

    /** Stops logging. Closes the output stream. */
    void stopLogging();

    /** Writes the values of the output variables to file. */
    void logToFile(Real time);
    

private:
    std::ofstream _output;  // the output file stream

    std::vector<LoggedVariable> _logged_variables;  // stores all the variables to be logged

    bool _logging = false;  // whether or not we have started logging. Once we start logging, prevent any new outputs from being added

    Real _logging_interval;
    Real _last_logging_time;
    

};

} // namespace Simulation