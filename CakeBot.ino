#include "Wire.h"
#include "Motor.h"

Motor leftMotor(2, 3);

void setup() {
  Serial.begin(38400);

  leftMotor.attach();
}

void loop() {
  Serial.println(leftMotor.getDt(), BIN);
}
/*
MPU6050 imu;

float prev_time, curr_time;

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
    Wire.begin();
    Serial.begin(38400);

    // Initialize I2C devices
    Serial.println("Starting I2C devices...");
    imu.initialize();

    // Verify connections
    Serial.println("Verifying connections...");
    Serial.println(imu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // Calculate accel data error
    if(gyro_error==0)
    {
      for(int i=0; i<200; i++)
      {
        Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68) 
        Wire.write(0x43);                        //First adress of the Gyro data
        Wire.endTransmission(false);
        Wire.requestFrom(0x68,4,true);           //We ask for just 4 registers 
           
        Gyr_rawX=Wire.read()<<8|Wire.read();     //Once again we shif and sum
        Gyr_rawY=Wire.read()<<8|Wire.read();

        Gyro_raw_error_x = Gyro_raw_error_x + (Gyr_rawX/32.8); 

        Gyro_raw_error_y = Gyro_raw_error_y + (Gyr_rawY/32.8);
        if(i==199)
        {
          Gyro_raw_error_x = Gyro_raw_error_x/200;
          Gyro_raw_error_y = Gyro_raw_error_y/200;
          gyro_error=1;
        }
      }
    }
}

void loop() {
    // Get IMU data
    imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Apply complementary filter to find angle
    angle_x = 0.98 * (angle_x + (gx * dt)) + 0.02 * ax;

    Serial.print("Gyro readings: ");
    Serial.print(angle_x);
    Serial.print("\n");
}*/
