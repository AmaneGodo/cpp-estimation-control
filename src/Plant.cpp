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
    true_state_.bias     = 0.2;
}

State Plant::update(double control_input, double dt) {
    double effective_accel = control_input - true_state_.bias;

    // Dummy deterministic update
    true_state_.position += true_state_.velocity * dt + 0.5 * effective_accel * dt * dt;
    true_state_.velocity += effective_accel * dt;
  
    // for learning purposes, add the noises inside the plant
    State measurement;
    measurement.position = true_state_.position + pos_noise_(rng_);
    measurement.velocity = true_state_.velocity + vel_noise_(rng_);
    measurement.bias     = 0.0; // bias is not measured

    // return the measurement as this will be seen by the estimator, not true state
    return measurement;
}

double Plant::getTrueBias() const {
    return true_state_.bias;
}