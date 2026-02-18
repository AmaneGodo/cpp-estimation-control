#ifndef PLANT_H
#define PLANT_H

#include "State.h"
#include <random>

class Plant {
public: 
    Plant();  
    State update(double control_input, double dt);

private:
    State true_state_;                              // keep true state hidden in the plant
    std::default_random_engine rng_;                //
    std::normal_distribution<double> pos_noise_;    // positional noise (normal distribution)
    std::normal_distribution<double> vel_noise_;    // velocity noise (normal distribution)
};  

#endif