#ifndef _MOTOR_H_
#define _MOTOR_H_

constexpr unsigned N_MOTORS = 1;

class Motor {
    public:
        Motor(byte forward, byte reverse,
              byte clk,  byte dt);
        void attach();
        void reset();
        void updateEncoder();
        double getVelocity();
        int getPosition();
        void setPosition(int position);
        void setDutyCycle(double dutyCycle);

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

        byte prev_enc;
        int prev_position;
        unsigned long prev_time;
        int position;
        double velocity;

        static Motor *self[N_MOTORS];
        static void (*const HANDLERS[N_MOTORS])();
};

#endif
