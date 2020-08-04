#include <Wire.h>
#include "Motor.h"
#include "PID_v1.h"
#include "MPU6050_tockn.h"

unsigned long sample_time = 100;
double angle_x = 0;
double wanted_angle_x = 0;
double output = 0;

MPU6050 imu(Wire);

Motor leftMotor(9, 10, 7, 8, // Ports
                125, -125, // Max / Min Output
                sample_time);

Motor rightMotor(5, 6, 2, 3, // Ports
                125, -125, // Max / Min Output
                sample_time);

PID pid = PID(&angle_x, &output, &wanted_angle_x, 0.26, 0, 0.03, DIRECT);

void setup() {
  Wire.begin();
  Serial.begin(9600);
 
  imu.begin();
  imu.calcGyroOffsets(true);

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-2, 2);

  leftMotor.setup();
  rightMotor.setup();
}

void loop() {
  imu.update();
  angle_x = imu.getAngleX();
  pid.Compute();
  //Serial.println(angle_x);
  Serial.println(output);
  leftMotor.set_velocity(output);
  rightMotor.set_velocity(-output);
  leftMotor.execute();
  rightMotor.execute();
}
