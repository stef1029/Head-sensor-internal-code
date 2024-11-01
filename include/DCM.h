#ifndef DCM_H
#define DCM_H

#include <Arduino.h>
#include <math.h>
// #include <cmath> // Include cmath for mathematical functions

namespace DCM {
    // Function declarations
    void Normalize();
    void Drift_correction();
    void Matrix_update();
    void Euler_angles();

    // Add more function declarations if needed
}

#endif // DCM_H