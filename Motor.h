#ifndef _MOTOR_H_
#define _MOTOR_H_

#ifdef ARDUINO
    #if ARDUINO < 100
        #include "WProgram.h"
    #else
        #include "Arduino.h"
    #endif
#endif

#include "PID_V1.h"
#include "Encoder.h"

constexpr int MIN_PWM = 10;

class Motor {
    public:
        Motor(byte forward, byte reverse,
              byte clk,  byte dt,
              double Kp, double Ki, double Kd,
              double min_output, double max_output,
              unsigned long sample_time);

        void setup();
        void execute();
        
        void reset_encoder();

        long get_position();
        double get_velocity();
        double get_wanted();
        double get_max();
        double get_min();

        void set_duty_cycle(double dutyCycle);
        void set_position(double position);
        void set_velocity(double wanted_velocity);
        void set_pid(double Kp, double Ki, double Kd);
        void set_limits(double max_output, double min_output);

    private:
        byte forward;
        byte reverse;
    
        double pwm_output;
        double max_output;
        double min_output;

        double prev_position;
        double curr_position;
        double wanted_position;

        double curr_velocity;
        double wanted_velocity;

        PID m_pid;
        Encoder m_enc;

        unsigned long prev_time;
        unsigned long sample_time;
};

#endif