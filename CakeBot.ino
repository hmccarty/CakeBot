#include "Wire.h"
#include "Motor.h"
//#include "PID.h"
#include "MPU6050.h"

unsigned long sample_time = 100;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int angle_x = 0;

unsigned long curr_time = 0;
unsigned long prev_time = 0;

MPU6050 imu;

Motor leftMotor(5, 6, 2, 3, // Ports
                255, -255, // Max / Min Output
                sample_time);
//
//PID leftMotorPID(leftMotor.get_pid_actual(),
//                 leftMotor.get_pid_goal(),
//                 0.00001, 0.0000, 0, //PID values
//                 leftMotor.get_max(), leftMotor.get_min(),
//                 sample_time);

void setup() {
  Wire.begin();
  Serial.begin(9600);

  imu.initialize();

  leftMotor.setup();
  //leftMotor.set_pid(&leftMotorPID);
  leftMotor.set_position(-3000);
}

void loop() {
  // Get IMU data
  //imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  //dt = curr_time - prev_time;

  // Apply complementary filter to find angle
  //angle_x = 0.98 * (angle_x + (gx * dt)) + 0.02 * ax;
//  Serial.print("Velocity: ");
  Serial.println(leftMotor.get_position());
//    Serial.println();
  leftMotor.execute();
}
