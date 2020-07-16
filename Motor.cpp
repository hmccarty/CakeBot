#include <Arduino.h>
#include "Motor.h"

const static byte qem[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

Motor *Motor::self[N_MOTORS] = {};
void (*const Motor::HANDLERS[N_MOTORS])() = {
    Motor::handler<0>
};

Motor::Motor(byte forward, byte reverse, byte clk, byte dt) {
    this->forward = forward;
    this->reverse = reverse;
   
    this->clk = clk;
    clk_mask = digitalPinToBitMask(clk);
    clk_port = portInputRegister(digitalPinToPort(clk));

    this->dt = dt;
    dt_mask = digitalPinToBitMask(dt);
    dt_port = portInputRegister(digitalPinToPort(dt));

    prev_enc = 0;
    prev_position = 0;
    position = 0;
    prev_time = 0;
    velocity = 0;
}

template <unsigned MOTOR_IDX>
static void Motor::handler() {
    self[MOTOR_IDX]->updateEncoder();
}

void Motor::attach() {
    self[0] = this;

    pinMode(forward, OUTPUT);
    pinMode(reverse, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(clk), HANDLERS[0], CHANGE);
    attachInterrupt(digitalPinToInterrupt(dt), HANDLERS[0], CHANGE);
}

void Motor::reset() {
    prev_enc = 0;
    position = 0;
    velocity = 0;
}

void Motor::updateEncoder() {
    uint8_t enc = ((*clk_port & clk_mask) << 1) | ((*dt_port & dt_mask) >> 5);
    prev_position = position;
    position += qem[(prev_enc << 2) | enc];
    prev_enc = enc;
    prev_time = millis();
}

void Motor::execute() {
  unsigned long curr_time = millis();
  unsigned long time_change = (curr_time - prev_time);

  if (time_change >= sample_time) {
    double curr_position = getPosition();
    double error = curr_position - prev_position;
    double p = kp * error;

    double output = p;

    if (output > max_output) {
      output = max_output;
    } else if (output < min_output) {
      output = min_output;
    }

    setDutyCycle(output);

    prev_position = curr_position;
    prev_time = curr_time;
  }
}

double Motor::getVelocity() {
    return velocity;
}

int Motor::getPosition() {
    return position;
}

void Motor::setPosition(int position) {
  double error = position - this->getPosition();
  
  this->setDutyCycle(error * 0.3);
}

void Motor::setDutyCycle(double dutyCycle) {
  if (dutyCycle < 0) {
    analogWrite(forward, 0);
    analogWrite(reverse, -dutyCycle);
  } else {
    analogWrite(forward, dutyCycle);
    analogWrite(reverse, 0);
  }
}
