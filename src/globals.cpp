#include "globals.h"
#include "Compass.h"
#include <Arduino.h>
#include <math.h>
#include "sensors.h"

// #include <cmath>

using namespace Math;
using namespace Globals;

namespace Globals {
    float accel[3];  // Definition without initialization
    float accel_min[3];
    float accel_max[3];

    float magnetom[3];
    float magnetom_min[3];
    float magnetom_max[3];
    float magnetom_tmp[3];

    float gyro[3];
    float gyro_average[3];
    int gyro_num_samples;

    float MAG_Heading;
    float Accel_Vector[3];
    float Gyro_Vector[3];
    float Omega_Vector[3];
    float Omega_P[3];
    float Omega_I[3];
    float Omega[3];
    float errorRollPitch[3];
    float errorYaw[3];
    float DCM_Matrix[3][3];
    float Update_Matrix[3][3];
    float Temporary_Matrix[3][3];

    float yaw;
    float pitch;
    float roll;

    unsigned long timestamp;
    unsigned long timestamp_old;
    float G_Dt;

    bool output_stream_on;
    bool output_single_on;
    int curr_calibration_sensor;
    bool reset_calibration_session_flag;
    int num_accel_errors;
    int num_magn_errors;
    int num_gyro_errors;

    int output_mode = 1;
    int output_format;
    bool output_errors;
    unsigned long message_id = 0;

    bool recording = false;  // Flag to indicate if recording is active
}

// Function definitions
void read_sensors() {
  Sensors::Read_Gyro(); // Read gyroscope
  Sensors::Read_Accel(); // Read accelerometer
  Sensors::Read_Magn(); // Read magnetometer
}

void reset_sensor_fusion() {
    float temp1[3];
    float temp2[3];
    float xAxis[] = {1.0f, 0.0f, 0.0f};

    read_sensors();
    timestamp = millis();
  
    // GET PITCH
    pitch = -atan2(Accel_Vector[0], sqrt(Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]));
	
    // GET ROLL
    Vector_Cross_Product(temp1, Accel_Vector, xAxis);
    Vector_Cross_Product(temp2, xAxis, temp1);
    roll = atan2(temp2[1], temp2[2]);
  
    // GET YAW
    Compass::Compass_Heading();
    yaw = MAG_Heading;
  
    // Init rotation matrix
    init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

void compensate_sensor_errors() {
    Accel_Vector[0] = (Accel_Vector[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    Accel_Vector[1] = (Accel_Vector[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    Accel_Vector[2] = (Accel_Vector[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

#if CALIBRATION__MAGN_USE_EXTENDED == true
    for (int i = 0; i < 3; i++) {
        magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
    }
    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
#else
    magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
    magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
#endif

    gyro[0] -= GYRO_AVERAGE_OFFSET_X;
    gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
    gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}

void check_reset_calibration_session() {
    if (!reset_calibration_session_flag) return;

    for (int i = 0; i < 3; i++) {
        accel_min[i] = accel_max[i] = Accel_Vector[i];
        magnetom_min[i] = magnetom_max[i] = magnetom[i];
    }

    gyro_num_samples = 0;
    gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;
    reset_calibration_session_flag = false;
}

void turn_output_stream_on() {
    output_stream_on = true;
    digitalWrite(STATUS_LED_PIN, HIGH);
}

void turn_output_stream_off() {
    output_stream_on = false;
    digitalWrite(STATUS_LED_PIN, LOW);
}

char readChar() {
    while (Serial.available() < 1) { }
    return Serial.read();
}
