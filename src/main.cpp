#include <iostream>
#include <fstream> // for file output

#include "Plant.h"
#include "Estimator.h"
#include "Estimator_Bias.h"
#include "Controller.h"

// Main control loop:
// Plant -> Estimator -> Controller -> Plant
// Mirrors a real estimation–control pipeline

int main() {
    double dt = 0.01;   // 10 ms timestep → 100 Hz loop
    double u = 0.0;     // control input

    double sigma_a = 0.2;
    double sigma_b = 0.01;
    double kp = 0.1;
    double kd = 0.05;
    double u_max = 2;
    double u_min = -2;

    bool enable_bias = true; // flip to false for baseline run
    
    const char* log_name = enable_bias ? "log_bias.csv" : "log_nominal.csv";
    std::ofstream log(log_name);

    std::cout << "System starting...\n";
    Plant plant;

    Estimator estimator(dt, sigma_a);
    Estimator_Bias estimator_bias(dt, sigma_a, sigma_b);

    Controller controller(kp, kd, u_max, u_min); 

    State measurement;
    State estimate;

    if (!log.is_open()) {
      std::cerr <<"Failed to open" << log_name << "\n";
      return 1;
    }

    if (enable_bias) {
      log << "step,t(s),meas_pos(m),meas_vel(m/s),true_bias(m/s^2),est_pos(m),est_vel(m/s),est_bias(m/s^2),u(m/s^2)\n";
    } else {
      log << "step,t(s),meas_pos(m),meas_vel(m/s),est_pos(m),est_vel(m/s),u(m/s^2)\n";
    }
    
    
    for (int k=0; k < 1000; ++k) {
        double t = k * dt;
        measurement = plant.update(u, dt);

        if (enable_bias) {
          estimate = estimator_bias.update(measurement, u);
        } else {
          estimate = estimator.update(measurement, u);
        }
        
        u = controller.update(estimate);

        log << k
            << "," << t
            << "," << measurement.position
            << "," << measurement.velocity;
        
            if (enable_bias) {
              log << "," << plant.getTrueBias();
            }
            
        log << "," << estimate.position
            << "," << estimate.velocity; 

        if (enable_bias) {
          log << "," << estimate.bias;
        }
            
        log << "," << u
            << "\n";

        if (k % 50 == 0) {
            std::cout << "\n--- step " << k << "---\n";
            std::cout << "[Plant] pos=" << measurement.position
              << " vel=" << measurement.velocity << "\n";
            std::cout << "[Estimator] est_pos=" << estimate.position
              << " est_vel=" << estimate.velocity;
            
            if (enable_bias) {
              std::cout << " est_bias=" << estimate.bias << "\n";
            } else {
              std::cout << "\n";
            }

            std::cout << "[Controller] u=" << u << "\n";
        }
    }

    return 0;
}
