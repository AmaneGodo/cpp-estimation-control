#include <iostream>
#include "Controller.h"

Controller::Controller(double kp, double kd, double u_max, double u_min)
    : kp_(kp), kd_(kd), u_max_(u_max), u_min_(u_min) {}
    // Control gains are stored as member variables so they persist
    // across updates and can be tuned independently of control logic.

double Controller::update(const State& estimated_state) {
    // Proportional-Derivative (PD) control
    // Goal: drive position toward zero while damping velocity to reduce overshoot

    // Control law:
    // u = -kp * position - kd * velocity
    //  - proportional term pulls position toward target
    //  - derivative term damps velocity to reduce overshoot
    double u = -kp_ * estimated_state.position - kd_ * estimated_state.velocity;

    // controller saturation
    // if (u > u_max_) u = u_max_;
    // if (u < u_min_) u = u_min_;
    // Saturation models actuator limits
    u = std::clamp(u, u_min_, u_max_);

    return u;
}