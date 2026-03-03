#include <iostream>
#include "Estimator_Bias.h"

Estimator_Bias::Estimator_Bias(double dt, double sigma_a, double sigma_b)
    : dt_(dt),
      sigma_a_(sigma_a),
      sigma_b_(sigma_b),
      initialized_(false)
{
    estimated_state_.position = 0.0;
    estimated_state_.velocity = 0.0;
    estimated_state_.bias = 0.2;

    P11_ = 1.0;
    P12_ = 0.0;
    P13_ = 0.0;
    P22_ = 1.0;
    P23_ = 0.0;
    P33_ = 1.0;

    R11_ = 0.05 * 0.05;
} 

State Estimator_Bias::update(const State& measurement, double u) {
    if (!initialized_) {
        estimated_state_.position = measurement.position;
        estimated_state_.velocity = 0.0;
        estimated_state_.bias     = 0.0;
        initialized_ = true;
        return estimated_state_;
    }

    const double dt = dt_;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt3 * dt;

    // prediction step
    const double effective_accel    = u - estimated_state_.bias;
    const double pos_predicted      = estimated_state_.position + estimated_state_.velocity * dt + 0.5 * effective_accel * dt2;
    const double vel_predicted      = estimated_state_.velocity + effective_accel * dt;
    const double bias_predicted     = estimated_state_.bias; 

    // Covariance prediction step: P_minus = P_predicted + Process Noise = A P A^T + Q
    // A * P
    // AP row 0
    const double AP11 = 1 * P11_ + dt * P12_ - 0.5 * dt2 * P13_;
    const double AP12 = 1 * P12_ + dt * P22_ - 0.5 * dt2 * P23_;
    const double AP13 = 1 * P13_ + dt * P23_ - 0.5 * dt2 * P33_;
    // AP row 1
    // const double AP21 = 1 * P12_ - dt * P13_;    // unused, syymetry check only
    const double AP22 = 1 * P22_ - dt * P23_;
    const double AP23 = 1 * P23_ - dt * P33_;
    // AP row 2
    // const double AP31 = 1 * P13_;                // unused, syymetry check only
    // const double AP32 = 1 * P23_;                // unused, syymetry check only
    const double AP33 = 1 * P33_;

    // AP * A^T
    // APA^T row 0
    double P11_minus = AP11 * 1 + AP12 * dt - AP13 * 0.5 * dt2;
    double P12_minus = AP12 * 1 - AP13 * dt;
    double P13_minus = AP13 * 1;
    // APA^T row 1
    // P21_minus = P12_minus
    double P22_minus = AP22 * 1 - AP23 * dt;
    double P23_minus = AP23 * 1;
    //APA^T row 2
    // P31_minus = P13_minus
    // P32_minus = P23_minus
    double P33_minus = AP33 * 1;

    // process noise Q:
    const double sa2 = sigma_a_ * sigma_a_;
    const double sb2 = sigma_b_ * sigma_b_;

    // Q row 0 and 1 => acceleration unceratinty: same as 2 state
    //Q row 2        => bias random walk
    // Q row 0
    const double Q11 = sa2 * (dt4 / 4);
    const double Q12 = sa2 * (dt3 / 2);
    // Q row 1
    // Q21 = Q12
    const double Q22 = sa2 * (dt2);
    // Q row 2
    const double Q33 = sb2 * (dt2);
    
    // full P_minus = P_minus + Q
    P11_minus += Q11;
    P12_minus += Q12;
    P22_minus += Q22;
    P33_minus += Q33;

    // Innovation
    const double innovation = measurement.position - pos_predicted;

    // innovation covariance
    // S = HP^-H^T + R where H = [1 0 0] where as [pos, vel, acc]
    // H * P_minus = [P11_minus  P12_minus  P13_minus]
    // H^T = [1
    //        0
    //        0] so: HP^-H^T = P11_minus
    const double S = P11_minus + R11_;
    
    // Kalman gains
    const double K1 = P11_minus / S;
    const double K2 = P12_minus / S;
    const double K3 = P13_minus / S;

    estimated_state_.position   = pos_predicted + K1 * innovation;
    estimated_state_.velocity   = vel_predicted + K2 * innovation;
    estimated_state_.bias       = bias_predicted + K3 * innovation;

    // Update covariance
    P11_ = P11_minus - K1 * P11_minus;
    P12_ = P12_minus - K1 * P12_minus;
    P13_ = P13_minus - K1 * P13_minus;

    P22_ = P22_minus - K2 * P12_minus;
    P23_ = P23_minus - K2 * P13_minus;

    P33_ = P33_minus - K3 * P13_minus;

    return estimated_state_;
}