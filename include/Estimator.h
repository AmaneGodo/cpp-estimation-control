#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "State.h"

class Estimator {
public: 
    Estimator(double dt, double sigma_a);
    State update(const State& measurement, double u);

private:
    State estimated_state_; 
    bool initialized_;                          // first measurement bbotstraps the estimator
    double dt_;                                 // time step
    double sigma_a_;                            // process accel noise std dev; tuning knob
    
    double P11_, P12_, P22_;                    // Covariance P (2x2, symmetric)

    double R11_, R22_;                          // measurement noise R (diagonal, since there is independent
};

#endif