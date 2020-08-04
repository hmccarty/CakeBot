#include "Motor.h"

const static byte qem[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

Motor *Motor::self[N_MOTORS] = {};
void (*const Motor::HANDLERS[N_MOTORS])() = {
    Motor::handler<0>
};

Motor::Motor(byte forward, byte reverse, byte clk, byte dt,
             double max_output, double min_output,
             unsigned long sample_time) 
  : enc(clk, dt)
{
    this->forward = forward;
    this->reverse = reverse;

    output = 0;
    this->max_output = max_output;
    this->min_output = min_output;

    prev_enc = 0;
    curr_position = 0;
    prev_position = 0;

    curr_velocity = 0;
    wanted_velocity = 0;

    prev_time = 0;
    this->sample_time = sample_time;
}

template <unsigned MOTOR_IDX>
static void Motor::handler() {
    self[MOTOR_IDX]->update_encoder();
}

void Motor::setup() {
    self[0] = this;

    pinMode(forward, OUTPUT);
    pinMode(reverse, OUTPUT);

    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(min_output, max_output);
}

void Motor::update_encoder() {
    byte enc = ((*clk_port & clk_mask) << 1) | ((*dt_port & dt_mask) >> 5);
    curr_position -= qem[(prev_enc << 2) | enc];
    prev_enc = enc;
}

void Motor::reset_encoder() {
    prev_enc = 0;
    curr_position = 0;
    prev_position = 0;
}

void Motor::execute() {
  unsigned long curr_time = millis();
  unsigned long time_change = (curr_time - prev_time);
  curr_position = (double) enc.read();

  if (time_change >= 50) {
    curr_velocity = ((curr_position - prev_position) / (double) time_change);

    prev_position = curr_position;
    prev_time = curr_time;
  }

  pid.Compute();
  //Serial.println(output);
  set_duty_cycle(output);
}

void Motor::set_duty_cycle(double dutyCycle) {
  if (abs(dutyCycle) < MIN_PWM) {
    dutyCycle = 0;
  }

  //Serial.println(dutyCycle);
  if (dutyCycle > 0) {
    analogWrite(forward, dutyCycle);
    analogWrite(reverse, 0);
  } else {
    analogWrite(forward, 0);
    analogWrite(reverse, -dutyCycle);
  }
}

long Motor::get_position() {
    return enc.read();
}

double Motor::get_velocity() {
    return curr_velocity;
}

void Motor::set_position(double position) {
  wanted_position = position;
}

double Motor::get_wanted() {
  return wanted_velocity;
}

double *Motor::get_pid_actual() {
  return &curr_velocity;
}

double *Motor::get_pid_goal() {
  return &wanted_velocity;
}

double Motor::get_max() {
  return max_output;
}

double Motor::get_min() {
  return min_output;
}

void Motor::set_velocity(double wanted_velocity) {
  this->wanted_velocity = wanted_velocity;
}
//
//void Motor::set_pid(PID *pid) {
//  this->pid = pid;
//}

void Motor::set_output(double max_output, double min_output) {
  this->max_output = max_output;
  this->min_output = min_output;
}
