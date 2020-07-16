#include "Wire.h"
#include "Motor.h"
#include "PID.h"
#include "MPU6050.h"

double sample_time = 10;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int angle_x = 0;

unsigned long curr_time = 0;
unsigned long prev_time = 0;

MPU6050 imu;

Motor leftMotor(5, 6, 2, 3, // Ports
                125, -125, // Max / Min Output
                sample_time);

PID leftMotorPID(leftMotor.get_pid_actual(),
                 leftMotor.get_pid_goal(),
                 0.1, 0, 0, //PID values
                 leftMotor.get_max(), leftMotor.get_min());

void setup() {
  Wire.begin();
  Serial.begin(38400);

  imu.initialize();

  leftMotor.setup();
  leftMotor.set_pid(&leftMotorPID);
  leftMotor.set_velocity(2000);
}

void loop() {
  // Get IMU data
  //imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  //dt = curr_time - prev_time;

  // Apply complementary filter to find angle
  //angle_x = 0.98 * (angle_x + (gx * dt)) + 0.02 * ax;
  
  Serial.println(leftMotor.get_velocity());
  leftMotor.execute();
}
