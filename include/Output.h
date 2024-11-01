#ifndef OUTPUT_H
#define OUTPUT_H

#include <Arduino.h>
#include "globals.h" // Include the global variables and constants
#include <math.h>

namespace Output {
    // Function declarations
    void output_angles(unsigned long message_id);
    void output_calibration(int calibration_sensor);
    void output_sensors_text(char raw_or_calibrated);
    void output_sensors_binary();
    void output_sensors();
}

#endif // OUTPUT_H