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
    const double innovation_velocity = measurement.velocity - predicted_velocity;
    
    // Innovation covariance - S
        // S = P_minus + R
    const double S11 = P11_minus + R11_;
    const double S12 = P12_minus;             // R off-diagonal is 0
    const double S22 = P22_minus + R22_;

    // invert S (2 x 2 inverse)
    const double det = S11 * S22 - S12 * S12;
    const double invS11 =  S22 / det;
    const double invS12 = -S12 / det;
    const double invS22 =  S11 / det;

    // Kalman Gain K = P_minus * S_inverse
    const double K11 = P11_minus * invS11 + P12_minus * invS12;
    const double K12 = P11_minus * invS12 + P12_minus * invS22;
    const double K21 = P12_minus * invS11 + P22_minus * invS12;
    const double K22 = P12_minus * invS12 + P22_minus * invS22;

    // updating estimated state: estimated state = predicted_state + kalman * innovation
    estimated_state_.position = predicted_position + K11 * innovation_position + K12 * innovation_velocity;
    estimated_state_.velocity = predicted_velocity + K21 * innovation_position + K22 * innovation_velocity;

    // Update covariance: P = (I - K) * P_minus
        // Since H = I, (I - KH) = (1 - K)
    const double IminusK11 = 1.0 - K11;
    const double IminusK12 = -K12;
    const double IminusK21 = -K21;
    const double IminusK22 = 1.0 - K22;

    P11_ = IminusK11 * P11_minus + IminusK12 * P12_minus;
    P12_ = IminusK11 * P12_minus + IminusK12 * P22_minus;
    P22_ = IminusK21 * P12_minus + IminusK22 * P22_minus;

    // P12_ stays as the off-diagonal; P21 is same

    return estimated_state_;
}
