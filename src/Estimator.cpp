#include <iostream>
#include "Estimator.h"

Estimator::Estimator(double alpha, double beta, double dt) 
    : alpha_(alpha),
      beta_(beta),
      dt_(dt),
      initialized_(false) // estimator starts uninformed
{
    estimated_state_.position = 0.0;
    estimated_state_.velocity = 0.0;
}

State Estimator::update(const State& measurement) {
    // 1. Initialization step
    if (!initialized_) {
        estimated_state_ = measurement;
        initialized_ = true;
        return estimated_state_;
    }

    // 2. Prediction step
    double predicted_position = estimated_state_.position + estimated_state_.velocity * dt_;
    double predicted_velocity = estimated_state_.velocity;

    // 3. Residual (innovation)
    double residual = measurement.position - predicted_position;

    // 4. Correction step
    estimated_state_.position = predicted_position + alpha_ * residual;
    estimated_state_.velocity = predicted_velocity + (beta_ / dt_) * residual;

    return estimated_state_;
}
