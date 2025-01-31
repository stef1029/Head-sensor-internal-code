#include "sensors.h"

using namespace Globals;

namespace Sensors {
    // Sensor I2C addresses
    const int ACCEL_ADDRESS = 0x53;
    const int MAGN_ADDRESS = 0x1E;
    const int GYRO_ADDRESS = 0x68;

    // Arduino backward compatibility macros
    #if ARDUINO >= 100
      #define WIRE_SEND(b) Wire.write((byte) b)
      #define WIRE_RECEIVE() Wire.read()
    #else
      #define WIRE_SEND(b) Wire.send(b)
      #define WIRE_RECEIVE() Wire.receive()
    #endif

    void I2C_Init() {
        Wire.begin();
    }

    void Accel_Init() {
        Wire.beginTransmission(ACCEL_ADDRESS);
        WIRE_SEND(0x2D);  // Power register
        WIRE_SEND(0x08);  // Measurement mode
        Wire.endTransmission();
        delay(5);
        Wire.beginTransmission(ACCEL_ADDRESS);
        WIRE_SEND(0x31);  // Data format register
        WIRE_SEND(0x08);  // Set to full resolution
        Wire.endTransmission();
        delay(5);
        
        // Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
        Wire.beginTransmission(ACCEL_ADDRESS);
        WIRE_SEND(0x2C);  // Rate
        WIRE_SEND(0x09);  // Set to 50Hz, normal operation
        Wire.endTransmission();
        delay(5);
    }

    void Read_Accel() {
        int i = 0;
        byte buff[6];
        
        Wire.beginTransmission(ACCEL_ADDRESS);
        WIRE_SEND(0x32);  // Send address to read from
        Wire.endTransmission();
        
        Wire.beginTransmission(ACCEL_ADDRESS);
        Wire.requestFrom(ACCEL_ADDRESS, 6);  // Request 6 bytes
        while (Wire.available()) {
            buff[i] = WIRE_RECEIVE();  // Read one byte
            i++;
        }
        Wire.endTransmission();
        
        if (i == 6) {  // All bytes received?
            accel[0] = (((int) buff[3]) << 8) | buff[2];  // X axis (internal sensor y axis)
            accel[1] = (((int) buff[1]) << 8) | buff[0];  // Y axis (internal sensor x axis)
            accel[2] = (((int) buff[5]) << 8) | buff[4];  // Z axis (internal sensor z axis)
        } else {
            num_accel_errors++;
            if (output_errors) Serial.println("!ERR: reading accelerometer");
        }
    }

    void Magn_Init() {
        Wire.beginTransmission(MAGN_ADDRESS);
        WIRE_SEND(0x02);
        WIRE_SEND(0x00);  // Set continuous mode (default 10Hz)
        Wire.endTransmission();
        delay(5);

        Wire.beginTransmission(MAGN_ADDRESS);
        WIRE_SEND(0x00);
        WIRE_SEND(0b00011000);  // Set 50Hz
        Wire.endTransmission();
        delay(5);
    }

    void Read_Magn() {
        int i = 0;
        byte buff[6];
        
        Wire.beginTransmission(MAGN_ADDRESS);
        WIRE_SEND(0x03);  // Send address to read from
        Wire.endTransmission();
        
        Wire.beginTransmission(MAGN_ADDRESS);
        Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
        while (Wire.available()) {
            buff[i] = WIRE_RECEIVE();  // Read one byte
            i++;
        }
        Wire.endTransmission();
        
        if (i == 6) {  // All bytes received?
            #if HW__VERSION_CODE == 10125
                magnetom[0] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // X axis (internal sensor -y axis)
                magnetom[1] = -1 * ((((int) buff[0]) << 8) | buff[1]);  // Y axis (internal sensor -x axis)
                magnetom[2] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // Z axis (internal sensor -z axis)
            #elif HW__VERSION_CODE == 10736
                magnetom[0] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // X axis (internal sensor -y axis)
                magnetom[1] = -1 * ((((int) buff[0]) << 8) | buff[1]);  // Y axis (internal sensor -x axis)
                magnetom[2] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)
            #elif (HW__VERSION_CODE == 10183) || (HW__VERSION_CODE == 10321)
                magnetom[0] = (((int) buff[0]) << 8) | buff[1];         // X axis (internal sensor x axis)
                magnetom[1] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // Y axis (internal sensor -y axis)
                magnetom[2] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // Z axis (internal sensor -z axis)
            #elif HW__VERSION_CODE == 10724
                magnetom[0] = (((int) buff[0]) << 8) | buff[1];         // X axis (internal sensor x axis)
                magnetom[1] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // Y axis (internal sensor -y axis)
                magnetom[2] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)
            #endif
        } else {
            num_magn_errors++;
            if (output_errors) Serial.println("!ERR: reading magnetometer");
        }
    }

    void Gyro_Init() {
        // Power up reset defaults
        Wire.beginTransmission(GYRO_ADDRESS);
        WIRE_SEND(0x3E);
        WIRE_SEND(0x80);
        Wire.endTransmission();
        delay(5);
        
        // Select full-scale range of the gyro sensors
        // Set LP filter bandwidth to 42Hz
        Wire.beginTransmission(GYRO_ADDRESS);
        WIRE_SEND(0x16);
        WIRE_SEND(0x1B);  // DLPF_CFG = 3, FS_SEL = 3
        Wire.endTransmission();
        delay(5);
        
        // Set sample rate to 50Hz
        Wire.beginTransmission(GYRO_ADDRESS);
        WIRE_SEND(0x15);
        WIRE_SEND(0x0A);  // SMPLRT_DIV = 10 (50Hz)
        Wire.endTransmission();
        delay(5);

        // Set clock to PLL with z gyro reference
        Wire.beginTransmission(GYRO_ADDRESS);
        WIRE_SEND(0x3E);
        WIRE_SEND(0x00);
        Wire.endTransmission();
        delay(5);
    }

    void Read_Gyro() {
        int i = 0;
        byte buff[6];
        
        Wire.beginTransmission(GYRO_ADDRESS);
        WIRE_SEND(0x1D);  // Send address to read from
        Wire.endTransmission();
        
        Wire.beginTransmission(GYRO_ADDRESS);
        Wire.requestFrom(GYRO_ADDRESS, 6);  // Request 6 bytes
        while (Wire.available()) {
            buff[i] = WIRE_RECEIVE();  // Read one byte
            i++;
        }
        Wire.endTransmission();
        
        if (i == 6) {  // All bytes received?
            gyro[0] = -1 * ((((int) buff[2]) << 8) | buff[3]);    // X axis (internal sensor -y axis)
            gyro[1] = -1 * ((((int) buff[0]) << 8) | buff[1]);    // Y axis (internal sensor -x axis)
            gyro[2] = -1 * ((((int) buff[4]) << 8) | buff[5]);    // Z axis (internal sensor -z axis)
        } else {
            num_gyro_errors++;
            if (output_errors) Serial.println("!ERR: reading gyroscope");
        }
    }
}