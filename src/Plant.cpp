#include <iostream>
#include "Plant.h"

Plant::Plant() 
    : pos_noise_(0.0, 0.05),
      vel_noise_(0.0, 0.02) {
    // Discrete-time plant dynamics:
    // The state is advanced before being printed, so step 0 reflects
    // the state after one update (not the raw initial condition).
    true_state_.position = 5.0; // initial disturbance
    true_state_.velocity = 1.0;
}

State Plant::update(double control_input, double dt) {
    // Dummy deterministic update
    true_state_.velocity += control_input * dt;
    true_state_.position += true_state_.velocity * dt;

    // for learning purposes, add the noises inside the plant
    State measurement;
    measurement.position = true_state_.position + pos_noise_(rng_);
    measurement.velocity = true_state_.velocity + vel_noise_(rng_);

    // return the measurement as this will be seen by the estimator, not true state
    return measurement;
}
