#include <iostream>
#include "Estimator.h"

Estimator::Estimator(double dt, double sigma_a) 
    : dt_(dt),
      sigma_a_(sigma_a),
      initialized_(false) // estimator starts uninformed
{
    estimated_state_.position = 0.0;
    estimated_state_.velocity = 0.0;

    // start with some uncertainty (tunable)
    P11_ = 1.0;
    P12_ = 0.0;
    P22_ = 1.0;

    // match R with plant noise 
        // R_pos = R11_ = pos noise ^2 (in plant = 0.05) = 0.0025
        // R_vel = R22_ = vel noise ^2 (in plant = 0.02) = 0.0004
    R11_ = 0.05 * 0.05;
    R22_ = 0.02 * 0.02;
}

State Estimator::update(const State& measurement, double u) {
    // 1. Initialization step
    if (!initialized_) {
        estimated_state_ = measurement;
        initialized_ = true;
        return estimated_state_;
    }

    const double dt = dt_;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;

    // State prediction step: Predicted state = A x + B u
        // A =  [1, dt]     B = [0.5*dt^2]
        //      [0,  1]         [   dt   ]
    const double predicted_position = estimated_state_.position + estimated_state_.velocity * dt + 0.5 * u * (dt * dt);
    const double predicted_velocity = estimated_state_.velocity + u * dt;

    // Covariance prediction step: P_minus = P_predicted + Process Noise = A P A^T + Q
        // A =  [1, dt]     
        //      [0,  1] 
    // P_predicted
    const double P11_predicted = P11_ + 2.0 * dt * P12_ + dt2 * P22_;
    const double P12_predicted = P12_ + dt * P22_;
    const double P22_predicted = P22_;

    // Process Noise
    // Q = (sigma_a)^2 * [(dt^4)/4, (dt^3)/2]
    //                   [(dt^3)/2,     dt^2]
    const double s2 = sigma_a_ * sigma_a_;
    const double Q11 = s2 * (dt4 / 4);
    const double Q12 = s2 * (dt3 / 2);
    const double Q22 = s2 * dt2;

    // predicted uncertainty P_minus
    const double P11_minus = P11_predicted + Q11;
    const double P12_minus = P12_predicted + Q12;
    const double P22_minus = P22_predicted + Q22;

    // Innovation state = measured state - predicted state
        // H = I
    const double innovation_position = measurement.position - predicted_position;
    
    // Innovation covariance - S
        // S = P_minus + R
    const double S = P11_minus + R11_;
    
    // Kalman Gain K = P_minus * S_inverse
    const double K1 = P11_minus / S;    // gaain for position state
    const double K2 = P12_minus / S;    // gain for velocity state (uses cross-variance)

    // updating estimated state: estimated state = predicted_state + kalman * innovation
    estimated_state_.position = predicted_position + K1 * innovation_position;
    estimated_state_.velocity = predicted_velocity + K2 * innovation_position;

    // Update covariance: P = (I - K) * P_minus
    P11_ = P11_minus - K1 * P11_minus;
    P12_ = P12_minus - K1 * P12_minus;
    P22_ = P22_minus - K2 * P12_minus;

    return estimated_state_;
}
