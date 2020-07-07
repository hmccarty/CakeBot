#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define PRId16 "hd"

MPU6050 imu;

int16_t ax, gy, az;
int16_t gx, gy, gz;

void setup() {
    Wire.begin();
    Serial.begin(38400);

    // Initialize I2C devices
    Serial.println("Starting I2C devices...");
    imu.Initialize();

    // Verify connections
    Serial.println("Verifying connections...");
    Serial.println(imu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
}

void loop() {
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    Serial.printf("Gyro readings: %" PRId16 ", %" PRId16 ", %" PRId16 "\n", gx, gy, gz);
}