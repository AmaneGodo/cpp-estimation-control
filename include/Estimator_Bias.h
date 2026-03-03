#ifndef ESTIMATOR_BIAS_H
#define ESTIMATOR_BIAS_H

#include "State.h"

class Estimator_Bias {
public:
    Estimator_Bias(double dt, double sigma_a, double sigma_b);
    State update(const State& measurement, double u);

private:
    State estimated_state_;
    bool initialized_;
    double dt_;

    double sigma_a_;
    double sigma_b_;

    double P11_, P12_, P13_, P22_, P23_, P33_;  // note: P12_ = P21_, P13_ = P31_, P23_ = P32_
    double R11_;                                // position variance
};
#endif