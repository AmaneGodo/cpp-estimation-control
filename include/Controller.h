#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "State.h"
#include <algorithm>

// Controller implements a simple PD control law.
// Control gains are owned by the Controller object and
// initialized once via the constructor.
class Controller {
public:
    // Constructor allows control gains to be configured at creation
    Controller(double kp , double kd , double u_max , double u_min);

    // Compute control input based on estimated state
    double update(const State& estimated_state);

private:
    double kp_;     // Proportional gain (position correction)
    double kd_;     // Derivative gain (velocity damping)
    double u_max_;  // Maximum control input limit
    double u_min_;  // Minimum control input limit
};

#endif