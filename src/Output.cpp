#include "output.h"
#include "CustomMath.h"
#include "globals.h" // Include the global variables and constants

using namespace Globals;
using namespace Math;

namespace Output {
    // void output_angles() {
    //     if (output_format == OUTPUT__FORMAT_BINARY) {
    //         float ypr[3];
    //         ypr[0] = TO_DEG(yaw);
    //         ypr[1] = TO_DEG(pitch);
    //         ypr[2] = TO_DEG(roll);
    //         Serial.write((byte*) ypr, 12);  // No new-line
    //     } else if (output_format == OUTPUT__FORMAT_TEXT) {
    //         Serial.print(millis());
    //         Serial.print("#YPR=");
    //         Serial.print(TO_DEG(yaw)); Serial.print(",");
    //         Serial.print(TO_DEG(pitch)); Serial.print(",");
    //         Serial.print(TO_DEG(roll)); Serial.println();
    //     }
    // }

    void output_angles(unsigned long message_id) {
        float ypr[3];
        ypr[0] = TO_DEG(yaw);
        ypr[1] = TO_DEG(pitch);
        ypr[2] = TO_DEG(roll);

        const byte startBoundary = 0x02; // Start of Text (STX) ASCII
        const byte endBoundary = 0x03;   // End of Text (ETX) ASCII

        Serial.write(startBoundary);

        Serial.write((byte*)&message_id, sizeof(message_id));

        Serial.write((byte*) ypr, sizeof(ypr));

        Serial.write(endBoundary);
    }

    void output_calibration(int calibration_sensor) {
        if (calibration_sensor == 0) {  // Accelerometer
            // Output MIN/MAX values
            Serial.print("accel x,y,z (min/max) = ");
            for (int i = 0; i < 3; i++) {
                if (accel[i] < accel_min[i]) accel_min[i] = accel[i];
                if (accel[i] > accel_max[i]) accel_max[i] = accel[i];
                Serial.print(accel_min[i]);
                Serial.print("/");
                Serial.print(accel_max[i]);
                if (i < 2) Serial.print("  ");
                else Serial.println();
            }
        } else if (calibration_sensor == 1) {  // Magnetometer
            // Output MIN/MAX values
            Serial.print("magn x,y,z (min/max) = ");
            for (int i = 0; i < 3; i++) {
                if (magnetom[i] < magnetom_min[i]) magnetom_min[i] = magnetom[i];
                if (magnetom[i] > magnetom_max[i]) magnetom_max[i] = magnetom[i];
                Serial.print(magnetom_min[i]);
                Serial.print("/");
                Serial.print(magnetom_max[i]);
                if (i < 2) Serial.print("  ");
                else Serial.println();
            }
        } else if (calibration_sensor == 2) {  // Gyroscope
            // Average gyro values
            for (int i = 0; i < 3; i++)
                gyro_average[i] += gyro[i];
            gyro_num_samples++;

            // Output current and averaged gyroscope values
            Serial.print("gyro x,y,z (current/average) = ");
            for (int i = 0; i < 3; i++) {
                Serial.print(gyro[i]);
                Serial.print("/");
                Serial.print(gyro_average[i] / (float) gyro_num_samples);
                if (i < 2) Serial.print("  ");
                else Serial.println();
            }
        }
    }

    void output_sensors_text(char raw_or_calibrated) {
        Serial.print("#A-"); Serial.print(raw_or_calibrated); Serial.print('=');
        Serial.print(accel[0]); Serial.print(",");
        Serial.print(accel[1]); Serial.print(",");
        Serial.print(accel[2]); Serial.println();

        Serial.print("#M-"); Serial.print(raw_or_calibrated); Serial.print('=');
        Serial.print(magnetom[0]); Serial.print(",");
        Serial.print(magnetom[1]); Serial.print(",");
        Serial.print(magnetom[2]); Serial.println();

        Serial.print("#G-"); Serial.print(raw_or_calibrated); Serial.print('=');
        Serial.print(gyro[0]); Serial.print(",");
        Serial.print(gyro[1]); Serial.print(",");
        Serial.print(gyro[2]); Serial.println();
    }

    // void output_sensors_binary() {
    //     Serial.write((byte*) accel, 12);
    //     Serial.write((byte*) magnetom, 12);
    //     Serial.write((byte*) gyro, 12);
    // }

    void output_sensors_binary() {
        // Example 2-byte "start of frame" marker
        const uint8_t header[2] = {0xAA, 0x55};

        // Write 2-byte header
        Serial.write(header, 2);

        // Then write the 36-byte sensor data (accel, magnetom, gyro)
        Serial.write((byte*)accel, 12);      // 3 floats => 12 bytes
        Serial.write((byte*)magnetom, 12);   // 3 floats => 12 bytes
        Serial.write((byte*)gyro, 12);       // 3 floats => 12 bytes
    }

    void output_sensors() {
        if (output_mode == OUTPUT__MODE_SENSORS_RAW) {
            if (output_format == OUTPUT__FORMAT_BINARY)
                output_sensors_binary();
            else if (output_format == OUTPUT__FORMAT_TEXT)
                output_sensors_text('R');
        } else if (output_mode == OUTPUT__MODE_SENSORS_CALIB) {
            // Apply sensor calibration
            compensate_sensor_errors();

            if (output_format == OUTPUT__FORMAT_BINARY)
                output_sensors_binary();
            else if (output_format == OUTPUT__FORMAT_TEXT)
                output_sensors_text('C');
        } else if (output_mode == OUTPUT__MODE_SENSORS_BOTH) {
            if (output_format == OUTPUT__FORMAT_BINARY) {
                output_sensors_binary();
                compensate_sensor_errors();
                output_sensors_binary();
            } else if (output_format == OUTPUT__FORMAT_TEXT) {
                output_sensors_text('R');
                compensate_sensor_errors();
                output_sensors_text('C');
            }
        }
    }
}