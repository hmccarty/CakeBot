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

constexpr unsigned N_MOTORS = 1;

class Motor {
    public:
        Motor(byte forward, byte reverse,
              byte clk,  byte dt,
              double max_output, double min_output,
              unsigned long sample_time);

        void setup();
        void execute();
        
        void reset_encoder();
        void update_encoder();

        void set_duty_cycle(double dutyCycle);

        long get_position();
        void set_position(double position);
        double get_velocity();
        double get_wanted();

        double *get_pid_actual();
        double *get_pid_goal();

        double get_max();
        double get_min();

        void set_velocity(double wanted_velocity);
        //void set_pid(PID *pid);
        void set_output(double max_output, double min_output);

        template <unsigned MOTOR_IDX> 
        static void handler();

    private:
        byte forward;
        byte reverse;
    
        byte clk;
        byte clk_mask;
        volatile byte *clk_port;

        byte dt;
        byte dt_mask;
        volatile byte *dt_port;

        double output;
        double max_output;
        double min_output;

        byte prev_enc;
        double curr_position;
        double wanted_position;
        double prev_position;

        double curr_velocity;
        double wanted_velocity;

        PID pid = PID(&curr_position, &output, &wanted_position, 0.1, 0.0001, 0, P_ON_E, DIRECT);
        Encoder enc = Encoder(2,3);

        unsigned long prev_time;
        unsigned long sample_time;

        static Motor *self[N_MOTORS];
        static void (*const HANDLERS[N_MOTORS])();
};

#endif
