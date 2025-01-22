/*
IMU mouse head sensor arduino code. Based on original IMU code that Ana used in her project.
Refactored for editing in Platformio by Stefan Rogers-Coltman (May 2024).
*/

#include <Wire.h>
#include <Arduino.h>
#include "globals.h"
#include "compass.h"
#include "DCM.h"
#include "CustomMath.h"
#include "sensors.h"
#include "output.h"

using namespace Globals;

void setup() {
    // Init serial output
    Serial.begin(OUTPUT__BAUD_RATE);

    // Init status LED
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    // Init sync pin
    pinMode(SYNC_PIN, OUTPUT);
    digitalWrite(SYNC_PIN, LOW);

    // Init sensors
    delay(50);  // Give sensors enough time to start
    Sensors::I2C_Init();
    Sensors::Accel_Init();
    Sensors::Magn_Init();
    Sensors::Gyro_Init();

    // Read sensors, init DCM algorithm
    delay(20);  // Give sensors enough time to collect data
    reset_sensor_fusion();
    Serial.flush();

    // Init output
#if (OUTPUT__HAS_RN_BLUETOOTH == true) || (OUTPUT__STARTUP_STREAM_ON == false)
    turn_output_stream_off();
#else
    turn_output_stream_on();
#endif
}

void loop() {
    // Check for any incoming messages from user:
    if (Serial.available() > 0) {
        delay(5);
        char command = Serial.read();
        if (command == 's') {  // Start recording
            recording = true;
            digitalWrite(STATUS_LED_PIN, HIGH);
            // Serial.println("Recording started.");
        } else if (command == 'e') {  // End recording
            recording = false;
            digitalWrite(STATUS_LED_PIN, LOW);
            // Serial.println("Recording stopped.");
        } else if (command == '#') {  // Start of new control message
            // delay(5);  // Wait for the message to be fully received
            command = Serial.read();  // Commands
            if (command == 'f') {  // request one output _f_rame
                output_single_on = true;
            } else if (command == 's') {  // _s_ynch request
                // Read ID
                byte id[2];
                id[0] = readChar();
                id[1] = readChar();

                // Reply with synch message
                Serial.print("#SYNCH");
                Serial.write(id, 2);
                Serial.println();
            } else if (command == 'o') {  // Set _o_utput mode
                char output_param = readChar();
                
                if (output_param == 'n') {  // Calibrate _n_ext sensor
                    curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
                    reset_calibration_session_flag = true;

                } else if (output_param == 't') {  // Output angles as _t_ext
                    output_mode = OUTPUT__MODE_ANGLES;
                    output_format = OUTPUT__FORMAT_TEXT;

                } else if (output_param == 'b') {  // Output angles in _b_inary format
                    output_mode = OUTPUT__MODE_ANGLES;
                    output_format = OUTPUT__FORMAT_BINARY;

                } else if (output_param == 'c') {  // Go to _c_alibration mode
                    output_mode = OUTPUT__MODE_CALIBRATE_SENSORS;
                    reset_calibration_session_flag = true;

                } else if (output_param == 's') {  // Output _s_ensor values
                    char values_param = readChar();
                    char format_param = readChar();
                    if (values_param == 'r')  // Output _r_aw sensor values
                        output_mode = OUTPUT__MODE_SENSORS_RAW;
                    else if (values_param == 'c')  // Output _c_alibrated sensor values
                        output_mode = OUTPUT__MODE_SENSORS_CALIB;
                    else if (values_param == 'b')  // Output _b_oth sensor values (raw and calibrated)
                        output_mode = OUTPUT__MODE_SENSORS_BOTH;
                        
                    if (format_param == 't')  // Output values as _t_ext
                        output_format = OUTPUT__FORMAT_TEXT;
                    else if (format_param == 'b')  // Output values in _b_inary format
                        output_format = OUTPUT__FORMAT_BINARY;

                } else if (output_param == '0') {  // Disable continuous streaming output
                    turn_output_stream_off();
                    reset_calibration_session_flag = true;

                } else if (output_param == '1') {  // Enable continuous streaming output
                    reset_calibration_session_flag = true;
                    turn_output_stream_on();

                } else if (output_param == 'e') {  // _e_rror output settings
                    char error_param = readChar();
                    if (error_param == '0') output_errors = false;
                    else if (error_param == '1') output_errors = true;
                    else if (error_param == 'c') {  // get error count
                        Serial.print("#AMG-ERR:");
                        Serial.print(num_accel_errors); Serial.print(",");
                        Serial.print(num_magn_errors); Serial.print(",");
                        Serial.println(num_gyro_errors);
                    }
                }
            }
#if OUTPUT__HAS_RN_BLUETOOTH == true
            // Read messages from bluetooth module
            // For this to work, the connect/disconnect message prefix of the module has to be set to "#".
            else if (command == 'C') // Bluetooth "#CONNECT" message (does the same as "#o1")
                turn_output_stream_on();
            else if (command == 'D') // Bluetooth "#DISCONNECT" message (does the same as "#o0")
                turn_output_stream_off();
#endif // OUTPUT__HAS_RN_BLUETOOTH == true
        } else { } // Skip character
    }

    // If recording is active, process sensor data
    if (recording) {
        // Time to read the sensors again?
        if ((millis() - timestamp) >= OUTPUT__DATA_INTERVAL) {
            timestamp_old = timestamp;
            timestamp = millis();
            if (timestamp > timestamp_old)
                G_Dt = (float)(timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
            else
                G_Dt = 0;

            // Update sensor readings
            read_sensors();

            if (output_mode == OUTPUT__MODE_CALIBRATE_SENSORS) {  // We're in calibration mode
                check_reset_calibration_session(); // Check if this session needs a reset
                if (output_stream_on || output_single_on)
                    Output::output_calibration(curr_calibration_sensor);
            } else if (output_mode == OUTPUT__MODE_ANGLES) { // Output angles
                // Apply sensor calibration
                compensate_sensor_errors();

                // Run DCM algorithm
                Compass::Compass_Heading(); // Calculate magnetic heading
                DCM::Matrix_update();
                DCM::Normalize();
                DCM::Drift_correction();
                DCM::Euler_angles();

                if (output_stream_on || output_single_on) {
                    // Raise sync pin and record the time
                    digitalWrite(SYNC_PIN, HIGH);
                    syncPinHighMillis = millis();
                    syncPinActive = true;

                    // Send angles out
                    Output::output_angles(message_id);
                    message_id++;
                }

            // if output mode is something other than angles or calibrate then do the below:
            } else { // Output sensor values
                if (output_stream_on || output_single_on) {
                    Output::output_sensors();
                }
            }



            output_single_on = false;

#if DEBUG__PRINT_LOOP_TIME == true
            Serial.print("loop time (ms) = ");
            Serial.println(millis() - timestamp);
#endif
        }
#if DEBUG__PRINT_LOOP_TIME == true
        else {
            Serial.println("waiting...");
        }
#endif
    }
        if (syncPinActive && (millis() - syncPinHighMillis >= SYNC_PULSE_DURATION_MS)) {
        digitalWrite(SYNC_PIN, LOW);
        syncPinActive = false;
    }
}
