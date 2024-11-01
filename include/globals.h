#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include "CustomMath.h"
#include <math.h>

// HARDWARE OPTIONS
// Select your hardware here by uncommenting one line!
//#define HW__VERSION_CODE 10125 // SparkFun "9DOF Razor IMU" version "SEN-10125" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10736 // SparkFun "9DOF Razor IMU" version "SEN-10736" (HMC5883L magnetometer)
//#define HW__VERSION_CODE 10183 // SparkFun "9DOF Sensor Stick" version "SEN-10183" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10321 // SparkFun "9DOF Sensor Stick" version "SEN-10321" (HMC5843 magnetometer)
#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)

// OUTPUT OPTIONS
#define OUTPUT__BAUD_RATE 57600
#define OUTPUT__DATA_INTERVAL 8  // in milliseconds

// Output mode definitions
#define OUTPUT__MODE_CALIBRATE_SENSORS 0
#define OUTPUT__MODE_ANGLES 1
#define OUTPUT__MODE_SENSORS_CALIB 2
#define OUTPUT__MODE_SENSORS_RAW 3
#define OUTPUT__MODE_SENSORS_BOTH 4

// Output format definitions
#define OUTPUT__FORMAT_TEXT 0
#define OUTPUT__FORMAT_BINARY 1

#define OUTPUT__STARTUP_STREAM_ON true  // true or false
#define OUTPUT__HAS_RN_BLUETOOTH false  // true or false

/*
Calibration:
Enter serial monitor and send #oc to enter calibration mode.
Freeze and start the values by sending #o0 and #o1 respectively. 
First is the accelerometer:
This should be done slowly, since you want to measure the gravity vector on each axis.
Move the board around slowly onto all axes, and note the min and max values for each axis.
Sudden movements will increase or decrease the gs too much.
Send #on to move sensor, which will now show the magn values.
Skip this for now.
Lay the sensor down on the table, and leave it still. 
Send #on to move to the gyro values. For this, leave the board still for 10 seconds as it measures the average noise.
Then freeze the values and take the second numbers for each axis and input these below. 
*/

// SENSOR CALIBRATION (Sensor from my desk values (20th May 2024 SRC))
// #define ACCEL_X_MIN ((float) -307)
// #define ACCEL_X_MAX ((float) 275)
// #define ACCEL_Y_MIN ((float) -283)
// #define ACCEL_Y_MAX ((float) 290)
// #define ACCEL_Z_MIN ((float) -322)
// #define ACCEL_Z_MAX ((float) 266)

// #define GYRO_AVERAGE_OFFSET_X ((float) -4.57)
// #define GYRO_AVERAGE_OFFSET_Y ((float) 47.67)
// #define GYRO_AVERAGE_OFFSET_Z ((float) 0)

// Ana's sensor values (22nd May 2024 Stefan Rogers-Coltman):
#define ACCEL_X_MIN ((float) -284.00)
#define ACCEL_X_MAX ((float) 270.00)
#define ACCEL_Y_MIN ((float) -281.00)
#define ACCEL_Y_MAX ((float) 272.00)
#define ACCEL_Z_MIN ((float) -327.00)
#define ACCEL_Z_MAX ((float) 232.00)


#define GYRO_AVERAGE_OFFSET_X ((float) 2.8)
#define GYRO_AVERAGE_OFFSET_Y ((float) 17.8)
#define GYRO_AVERAGE_OFFSET_Z ((float) -15.26)

#define CALIBRATION__MAGN_USE_EXTENDED true
const float magn_ellipsoid_center[3] = {-37.5993, -316.192, 185.420};
const float magn_ellipsoid_transform[3][3] = {
    {0.942895, -0.0298951, -0.0180144},
    {-0.0298951, 0.815543, 0.0558301},
    {-0.0180144, 0.0558301, 0.969087}
};



#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

#define GYRO_GAIN 0.06957
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN))

#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

#define STATUS_LED_PIN 13
#define SYNC_PIN 6

#define GRAVITY 256.0f
#define TO_RAD(x) (x * 0.01745329252)
#define TO_DEG(x) (x * 57.2957795131)

#define DEBUG__NO_DRIFT_CORRECTION false
#define DEBUG__PRINT_LOOP_TIME false

// Global variables
namespace Globals {

    // Sensor variables
    extern float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
    extern float accel_min[3];
    extern float accel_max[3];

    extern float magnetom[3];
    extern float magnetom_min[3];
    extern float magnetom_max[3];
    extern float magnetom_tmp[3];

    extern float gyro[3];
    extern float gyro_average[3];
    extern int gyro_num_samples;

    extern float MAG_Heading;
    extern float Accel_Vector[3];
    extern float Gyro_Vector[3];
    extern float Omega_Vector[3];
    extern float Omega_P[3];
    extern float Omega_I[3];
    extern float Omega[3];
    extern float errorRollPitch[3];
    extern float errorYaw[3];
    extern float DCM_Matrix[3][3];
    extern float Update_Matrix[3][3];
    extern float Temporary_Matrix[3][3];

    extern float yaw;
    extern float pitch;
    extern float roll;

    extern unsigned long timestamp;
    extern unsigned long timestamp_old;
    extern float G_Dt;

    extern bool output_stream_on;
    extern bool output_single_on;
    extern int curr_calibration_sensor;
    extern bool reset_calibration_session_flag;
    extern int num_accel_errors;
    extern int num_magn_errors;
    extern int num_gyro_errors;

    extern int output_mode;
    extern int output_format;
    extern bool output_errors;
    extern unsigned long message_id;

    extern bool recording;  // Flag to indicate if recording is active
}

// Compass namespace for Compass-related functions
namespace Compass {
    void Compass_Heading();
}

// Function declarations
void read_sensors();
void reset_sensor_fusion();
void compensate_sensor_errors();
void check_reset_calibration_session();
void turn_output_stream_on();
void turn_output_stream_off();
char readChar();

#endif // GLOBALS_H