#include <Arduino.h>
#include "Motor.h"

Motor::Motor(uint8_t clk, uint8_t dt) {
    this->clk = clk;
    clk_mask = digitalPinToBitMask(clk);
    clk_port = portInputRegister(digitalPinToPort(clk));

    this->dt = dt;
    dt_mask = digitalPinToBitMask(dt);
    dt_port = portInputRegister(digitalPinToPort(dt));
}

void Motor::setup() {
    attachInterrupt(digitalPinToInterrupt(clk), this->update, CHANGE);
    attachInterrupt(digitalPinToInterrupt(dt), this->update, CHANGE);

    prev_enc = 0;
    position = 0;
    velocity = 0;
}

void Motor::reset() {
    position = 0;
    velocity = 0;
}

void Motor::update() {
    static uint8_t qem[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
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

uint8_t getDt() {
    return (*clk_port & clk_mask);
}
