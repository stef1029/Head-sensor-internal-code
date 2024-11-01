#include "globals.h"
#include "Compass.h"
#include <Arduino.h>
#include <math.h>
// #include <cmath> // Include cmath for atan2 and cos/sin functions

void Compass::Compass_Heading() {
    float mag_x;
    float mag_y;
    float cos_roll;
    float sin_roll;
    float cos_pitch;
    float sin_pitch;
  
    cos_roll = cos(Globals::roll);
    sin_roll = sin(Globals::roll);
    cos_pitch = cos(Globals::pitch);
    sin_pitch = sin(Globals::pitch);
  
    // Tilt compensated magnetic field X
    mag_x = Globals::magnetom[0] * cos_pitch + Globals::magnetom[1] * sin_roll * sin_pitch + Globals::magnetom[2] * cos_roll * sin_pitch;
    // Tilt compensated magnetic field Y
    mag_y = Globals::magnetom[1] * cos_roll - Globals::magnetom[2] * sin_roll;
    // Magnetic Heading
    Globals::MAG_Heading = atan2(-mag_y, mag_x);
}