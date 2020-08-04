#include <Wire.h>
#include "Motor.h"
#include "PID_v1.h"
#include "MPU6050.h"

unsigned long sample_time = 100;

double angle = 0;
double wanted_angle = 0;
double output = 0;

MPU6050 imu(Wire);

Motor leftMotor(9, 10, 7, 8, // Fwd, Rvs, Clk, Dt
                -125, 125, // Min / Max Output
                23, 8, 0, // Kp, Ki, Kd
                sample_time);

Motor rightMotor(6, 5, 2, 3, // Fwd, Rvs, Clk, Dt
                -125, 125, // Min / Max Output
                23, 8, 0, // Kp, Ki, Kd
                sample_time);

PID pid = PID(&angle, &output, &wanted_angle, // Input, Output, Setpoint
              0.26, 0, 0.03, DIRECT); // Kp, Ki, Kd, POn

void setup() {
  Wire.begin();
  Serial.begin(9600);
 
  imu.begin();
  imu.calcGyroOffsets(true);

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-2, 2); // Min, Max

  leftMotor.setup();
  rightMotor.setup();
}

void loop() {
  imu.update();
  angle = imu.getAngleX();

  pid.Compute();
  leftMotor.set_velocity(output);
  rightMotor.set_velocity(output);

  leftMotor.execute();
  rightMotor.execute();
}