#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include "globals.h" // Include the global variables and constants
#include <math.h>

namespace Sensors {
    // I2C initialization
    void I2C_Init();
    
    // Accelerometer functions
    void Accel_Init();
    void Read_Accel();
    
    // Magnetometer functions
    void Magn_Init();
    void Read_Magn();
    
    // Gyroscope functions
    void Gyro_Init();
    void Read_Gyro();
}

#endif // SENSORS_H