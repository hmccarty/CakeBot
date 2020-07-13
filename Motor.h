#ifndef _MOTOR_H_
#define _MOTOR_H_

class Motor {
    public:
        Motor(uint8_t clk,  uint8_t dt);
        void setup();
        int32_t getVelocity();
        int32_t getPosition();
        void update();
    private:
        uint8_t clk;
        uint8_t dt;

        uint8_t prev_enc;
        
        int32_t position;
        int32_t velocity;

        uint32_t prev_time;
}

#endif