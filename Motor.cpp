#include <Arduino.h>
#include "Motor.h"

Motor::Motor(uint8_t enc_a, uint8_t enc_b) {
    this->enc_a = enc_a;
    this->enc_b = enc_b;
}

Motor::setup() {
    pinMode(enc_a, INPUT);
    pinMode(enc_b, INPUT);

    position = 0;
    velocity = 0;

    prev_time = 0;
}

void Motor::reset() {
    position = 0;
    velocity = 0;
}

int32_t Motor::getVelocity() {
    return velocity;
}

int32_t Motor::getPosition() {
    return position;
}

void Motor::update() {
    
}