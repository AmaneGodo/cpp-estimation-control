#include <iostream>
#include <fstream> // for file output

#include "Plant.h"
#include "Estimator.h"
#include "Controller.h"

// Main control loop:
// Plant -> Estimator -> Controller -> Plant
// Mirrors a real estimation–control pipeline

int main() {
    double dt = 0.01;   // 10 ms timestep → 100 Hz loop

    std::cout << "System starting...\n";
    Plant plant;
    Estimator estimator(0.2, 0.005, dt);
    Controller controller(0.1, 0.05, 2, -2); // (kp, kd, u_max, u_min)

    double u = 0.0;     // control input

    State measurement;
    State estimate;

    // for logging
    std::ofstream log("log.csv");

    if (!log.is_open()) {
      std::cerr <<"Failed to open log.csv\n";
      return 1;
    }

    log << "step,t(s),meas_pos(m),meas_vel(m/s),est_pos(m),est_vel(m/s),u(m/s^2)\n";
    
    for (int k=0; k < 1000; ++k) {
        double t = k * dt;
        measurement   = plant.update(u, dt);
        estimate      = estimator.update(measurement);
        u             = controller.update(estimate);

        log << k
            << "," << t
            << "," << measurement.position
            << "," << measurement.velocity
            << "," << estimate.position
            << "," << estimate.velocity 
            << "," << u
            << "\n";

        if (k % 50 == 0) {
            std::cout << "\n--- step " << k << "---\n";
            std::cout << "[Plant] pos=" << measurement.position
              << " vel=" << measurement.velocity << "\n";
            std::cout << "[Estimator] est_pos=" << estimate.position
              << " est_vel=" << estimate.velocity << "\n";
            std::cout << "[Controller] u=" << u << "\n";
        }
    }

    return 0;
}
