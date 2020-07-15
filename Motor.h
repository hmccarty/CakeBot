#ifndef _MOTOR_H_
#define _MOTOR_H_

constexpr unsigned N_MOTORS = 1;

class Motor {
    public:
        Motor(uint8_t clk,  uint8_t dt);
        void attach();
        void reset();
        void update();
        int32_t getVelocity();
        int32_t getPosition();
        uint8_t getDt();

        template <unsigned PIN> 
        static void handler();

    private:
        uint8_t clk;
        uint8_t clk_mask;
        volatile uint8_t *clk_port;

        uint8_t dt;
        uint8_t dt_mask;
        volatile uint8_t *dt_port;

        uint8_t prev_enc;
        int32_t position;
        int32_t velocity;

        static Motor *self[N_MOTORS];
        static void (*const HANDLERS[N_MOTORS])();
};

#endif
