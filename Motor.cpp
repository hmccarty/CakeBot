#include <Arduino.h>
#include "Motor.h"

Motor *Motor::self[N_MOTORS] = {};
void (*const Motor::HANDLERS[N_MOTORS])() = {
    Motor::handler<0>
};

Motor::Motor(uint8_t clk, uint8_t dt) {
    this->clk = clk;
    clk_mask = digitalPinToBitMask(clk);
    clk_port = portInputRegister(digitalPinToPort(clk));

    this->dt = dt;
    dt_mask = digitalPinToBitMask(dt);
    dt_port = portInputRegister(digitalPinToPort(dt));

    prev_enc = 0;
    position = 0;
    velocity = 0;
}

template <unsigned MOTOR_IDX>
static void Motor::handler() {
    self[MOTOR_IDX]->update();
}

void Motor::attach() {
    self[clk] = this;
    self[dt] = this;

    attachInterrupt(digitalPinToInterrupt(clk), HANDLERS[clk], CHANGE);
    attachInterrupt(digitalPinToInterrupt(dt), HANDLERS[dt], CHANGE);
}

void Motor::reset() {
    prev_enc = 0;
    position = 0;
    velocity = 0;
}

void Motor::update() {
    const static int8_t qem[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    uint8_t enc = ((*clk_port & clk_mask) << 1) | ((*dt_port & dt_mask) >> 5);
    position += qem[(prev_enc << 2) | enc];
    prev_enc = enc;
}

int32_t Motor::getVelocity() {
    return velocity;
}

int32_t Motor::getPosition() {
    return position;
}

uint8_t Motor::getDt() {
    return (*clk_port & clk_mask);
}
