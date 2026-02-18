#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "State.h"

class Estimator {
public: 
    Estimator(double alpha, double beta, double dt);
    State update(const State& measurement);

private:
    State estimated_state_; 
    
    bool initialized_;  // first measurement bbotstraps the estimator
    
    double alpha_;      // estimator tuning positional gain
    double beta_;       // estimator tuning velocity gain
    double dt_;         // time step
};

#endif